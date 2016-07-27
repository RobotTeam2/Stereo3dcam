#include <stdio.h>
#include <unistd.h>

#include "r2_chassis.h"

long requestSetSpeed, requestMove, requestStop, requestDebugWheelLeft, requestDebugWheelRight;

#define ALL_STEPS (15+36+36+15)
long count = ALL_STEPS;

static void listener(long requestID, r2_chassis_event_t eventType, void *eventArg)
{
#if 0
	if (requestID == requestSetSpeed) {
		printf("setSpeed");
	}
	else if (requestID == requestMove) {
		printf("move");
	}
	else if (requestID == requestStop) {
		printf("stop");
	}
	else if (requestID == requestDebugWheelLeft) {
		printf("debugWheel left");
	}
	else if (requestID == requestDebugWheelRight) {
		printf("debugWheel right");
	}
	else {
		return;
	}

	printf(" ");
#endif

	switch (eventType) {
	case R2_CHASSIS_EVENT_UNKNOWN:
		printf("unknown event");
		break;
	case R2_CHASSIS_EVENT_INFORMATION:
		printf("information");
		break;
	case R2_CHASSIS_EVENT_ACCEPT:
		printf("accepted");
		break;
	case R2_CHASSIS_EVENT_COMPLETE:
		printf("completed");
		if (count == ALL_STEPS) {
			printf("\nsleep 10");
			sleep(10);
		}
		else {
			usleep(1000000);
		}
		if (count == ALL_STEPS) {
			requestMove = r2_chassis_move(R2_CHASSIS_DIR_FORWARD, 15);
			count -= 15;
		}
		else if (count <= ALL_STEPS - 15 && count >= ALL_STEPS - 15 - 36) {
			requestMove = r2_chassis_move(R2_CHASSIS_DIR_TURN_LEFT, 1);
			count--;
		}
		else if (count <= ALL_STEPS - 15 - 36 && count >= ALL_STEPS - 15 - 36 - 36) {
			requestMove = r2_chassis_move(R2_CHASSIS_DIR_TURN_RIGHT, 1);
			count--;
		}
		else if (count > 0) {
			requestMove = r2_chassis_move(R2_CHASSIS_DIR_BACKWARD, 15);
			count -= 15;
		}
		break;
	case R2_CHASSIS_EVENT_ABORT:
		printf("aborted");
		break;
	}

	printf("\n");
}

int main()
{
	r2_chassis_init("/dev/ttyUSB0");
	r2_chassis_setListener(listener);
	sleep(10);
	r2_chassis_on();
	sleep(2);

	requestSetSpeed = r2_chassis_setSpeed(255);
	sleep(1);

	requestMove = r2_chassis_move(R2_CHASSIS_DIR_TURN_LEFT, 10);
	sleep(5);
	requestStop = r2_chassis_stop();
	sleep(5);
	requestDebugWheelLeft = r2_chassis_debug_wheel(0, 10);
	requestDebugWheelRight = r2_chassis_debug_wheel(1, -10);
	sleep(5);

	while (count > 0) {
		sleep(1);
	}

	sleep(10);
	r2_chassis_off();
	sleep(2);

	r2_chassis_finalize();
	return 0;
}
