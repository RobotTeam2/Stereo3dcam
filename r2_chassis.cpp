#include "r2_chassis.h"

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>

#define MAX_WRITE_BUFFER        1024
#define MAX_READ_BUFFER         2048

char strCommand[MAX_WRITE_BUFFER] = { '\0' };
long g_requestID = 0;
r2_chassis_listener_t g_listener = NULL;

int fd;
pthread_t reader_thread;
struct termios g_oldtio;

static void *reader(void *);
static void ProcessResponse(char *lpBuf, size_t dwRead);

int r2_chassis_init(const char *device)
{
	struct termios newtio;

	fd = open(device, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		perror(device);
		return errno;
	}

	tcgetattr(fd, &g_oldtio);
	newtio = g_oldtio;
	//bzero(&newtio, sizeof(newtio));

	cfsetispeed(&newtio, B115200);
	cfsetospeed(&newtio, B115200);
	newtio.c_cflag |= CLOCAL;

	tcflush(fd, TCIOFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);

	if (pthread_create(&reader_thread, NULL, reader, NULL) < 0) {
		perror("pthread_create");
		return errno;
	}

	return 0;
}

void r2_chassis_finalize(void)
{
	pthread_kill(reader_thread, SIGTERM);
	/* TODO: cancel */
	//pthread_join(reader_thread, NULL);

	tcsetattr(fd, TCSANOW, &g_oldtio);
	close(fd);
}

int r2_chassis_on(void)
{
	snprintf(strCommand, sizeof(strCommand) - 1, "%ld srv_on\n", ++g_requestID);
	write(fd, strCommand, strlen(strCommand));
	return g_requestID;
}

int r2_chassis_off(void)
{
	snprintf(strCommand, sizeof(strCommand) - 1, "%ld srv_off\n", ++g_requestID);
	write(fd, strCommand, strlen(strCommand));
	return g_requestID;
}

void r2_chassis_setListener(r2_chassis_listener_t listener)
{
	g_listener = listener;
}

long r2_chassis_move(r2_chassis_dir_t dir, int steps)
{
	snprintf(strCommand, sizeof(strCommand) - 1, "%ld move %d %d\n", ++g_requestID, dir, steps);
	write(fd, strCommand, strlen(strCommand));
	return g_requestID;
}

long r2_chassis_setSpeed(int speed)
{
	snprintf(strCommand, sizeof(strCommand) - 1, "%ld setSpeed %d\n", ++g_requestID, speed);
	write(fd, strCommand, strlen(strCommand));
	return g_requestID;
}

long r2_chassis_stop(void)
{
	snprintf(strCommand, sizeof(strCommand) - 1, "%ld stop\n", ++g_requestID);
	write(fd, strCommand, strlen(strCommand));
	return g_requestID;
}

long r2_chassis_debug_wheel(int wheelNo, int steps)
{
	snprintf(strCommand, sizeof(strCommand) - 1, "%ld debug_wheel %d %d\n", ++g_requestID, wheelNo, steps);
	write(fd, strCommand, strlen(strCommand));
	return g_requestID;
}

static void *reader(void *arg)
{
	char buf[255];
	int res;

	fd_set readfs;
	int maxfd = fd + 1;

	while (1) {
		FD_ZERO(&readfs);
		FD_SET(fd, &readfs);
		select(maxfd, &readfs, NULL, NULL, NULL);
		if (FD_ISSET(fd, &readfs)) {
			res = read(fd, buf, 255);
			if (res < 0) {
				perror("read");
				return (void *)errno;
			}
			ProcessResponse(buf, res);
		}
	}
	return (void *)0;
}

static void ProcessResponse(char *lpBuf, size_t dwRead)
{
	static char respBuf[MAX_READ_BUFFER] = { '\0' };
	char *eol;
	r2_chassis_event_t eventType = R2_CHASSIS_EVENT_UNKNOWN;
	long requestID;

	if (!g_listener) {
		// do nothing
		return;
	}

	strncat(respBuf, lpBuf, dwRead);
	eol = strchr(respBuf, '\n');
	if (eol == NULL) {
		return;
	}
	else if(eol - respBuf < 3) {
		memmove(&respBuf[0], eol + 1, strlen(eol + 1) + 1);
		return;
	}
	else {
		*eol = '\0';
	}

	printf("[DEBUG]%s\n", respBuf);
	switch (respBuf[0]) {
	case 'A':
		/* Accepted */
		eventType = R2_CHASSIS_EVENT_ACCEPT;
		break;
	case 'C':
		/* Completed */
		eventType = R2_CHASSIS_EVENT_COMPLETE;
		break;
	case 'Q':
		/* Aborted */
		eventType = R2_CHASSIS_EVENT_ABORT;
		break;
	case 'I':
		/* Information */
		eventType = R2_CHASSIS_EVENT_INFORMATION;
		break;
	}

	sscanf(&respBuf[2], "%ld", &requestID);
	g_listener(requestID, eventType, NULL);

	memmove(&respBuf[0], eol + 1, strlen(eol + 1) + 1);
}
