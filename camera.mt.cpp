#include <stdio.h>
#include <string>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
#include <libusb-1.0/libusb.h>
#include <libuvc/libuvc.h>
#include <libuvc/libuvc_config.h>
#include "libuvc/include/libuvc/libuvc_internal.h"

#define SHOW_WINDOW
#define VGA_WIDTH 640
#define VGA_HEIGHT 480

#define COUNT_THR (768*5) //5% of 320x240
#define DIFFDIST 900.0


using namespace std;

void smd_cmd(uvc_device_handle_t *devh);

uvc_context_t *ctx=NULL;
uvc_device_t *dev=NULL;
uvc_device_handle_t *devh=NULL;
uvc_stream_ctrl_t ctrl;
uvc_error_t res=(uvc_error_t)0;

IplImage* gOrigImage= cvCreateImage(cvSize(VGA_WIDTH,VGA_HEIGHT),8,3);;
uvc_frame_t *gBGRFrame = uvc_allocate_frame(VGA_WIDTH * VGA_HEIGHT * 3);

IplImage* gLeftImage = cvCreateImage(cvSize(VGA_WIDTH/2,VGA_HEIGHT/2),8,3);
IplImage* gGrayLeftImage = cvCreateImage(cvGetSize(gLeftImage),8,1);

IplImage* gRightImage = cvCreateImage(cvSize(VGA_WIDTH/2,VGA_HEIGHT/2),8,3);
IplImage* gGrayRightImage = cvCreateImage(cvGetSize(gLeftImage),8,1);

IplImage* gTempImage = NULL;
IplImage* gFliterImage = cvCreateImage(cvGetSize(gLeftImage),8,3);

IplImage* gDepthImage = cvCreateImage(cvGetSize(gGrayLeftImage),8,3);
IplImage* gGrayDepthImage = cvCreateImage(cvGetSize(gGrayLeftImage),8,1);

IplImage* gThrDepthImage = cvCreateImage(cvGetSize(gLeftImage),8,3);
IplImage* gGrayThrDepthImage = cvCreateImage(cvGetSize(gGrayLeftImage),8,1);

bool drawFlag=false;
bool cbFlag=false;
  
void cb(uvc_frame_t *frame, void *ptr)
{

    uvc_error_t ret=(uvc_error_t)0;

    cbFlag=true;
//  printf("callback! length = %u, ptr = %d\n", (unsigned int)frame->data_bytes, (long)ptr);
  
    if(drawFlag == true) return;

    ret = uvc_any2bgr(frame, gBGRFrame);
    if(ret)uvc_perror(ret, "uvc_any2bgr");

}

uvc_error_t initCamera(void)
{
    uvc_error_t res=(uvc_error_t)0;
    
	res = uvc_init(&ctx, NULL);
    if (res < 0){
        uvc_perror(res, "uvc_init");
        return res;
    }
    puts("UVC initialized");

    res = uvc_find_device(ctx, &dev,0, 0, NULL);
    if (res < 0) {
        uvc_perror(res, "uvc_find_device"); /* no devices found */
        return res;
    } else {
        puts("Device found");
    }

    res = uvc_open(dev, &devh);
    if (res < 0) {
        uvc_perror(res, "uvc_open"); /* unable to open device */
        return res;
    } else {
        puts("Device opened");
        uvc_print_diag(devh, stderr);
    }

    res = uvc_get_stream_ctrl_format_size(devh, &ctrl,UVC_FRAME_FORMAT_YUYV,VGA_WIDTH, VGA_HEIGHT, 30);
    uvc_print_stream_ctrl(&ctrl, stderr);
      
    if(res < 0){
        uvc_perror(res, "get_mode");
        return res;
    }
    
    res = uvc_start_streaming(devh, &ctrl, cb, (void *)12345, 0);
    if(res < 0){
        uvc_perror(res, "start_streaming");
        return res;
    }
    
    return res;
}

void smd_cmd(uvc_device_handle_t *devh)
{
    char buf[2];
    int res=0;
    libusb_device_handle *dev_handle=NULL;

    buf[0] = 0x76;
    buf[1] = 0xc3;
    dev_handle = devh->usb_devh;
    
    res = libusb_claim_interface(dev_handle, 0);
    if(res != LIBUSB_SUCCESS) printf("claim 0 is failed!\n");
    res = libusb_control_transfer(devh->usb_devh,0x21,0x01,0x0800,0x0600,(unsigned char *)buf,sizeof(buf),0);
    printf("res = %d\n",res);
    if(res == sizeof(buf))
        printf("Control transfer Success!\n");
    else 
        printf("Control transfer Failed!\n");


    buf[0] = 0x04;
    buf[1] = 0x00;
    res = libusb_control_transfer(devh->usb_devh,0x21,0x01,0xa00,0x600,(unsigned char *)buf,sizeof(buf),0);
    if(res == sizeof(buf))
        printf("Control transfer Success!\n");
    else 
        printf("Control transfer Failed!\n");
}

void getDepthImage(void)
{
	gTempImage=cvCloneImage(gOrigImage);
	
	cvSetImageROI(gTempImage,cvRect(0,          VGA_HEIGHT/4,VGA_WIDTH/2,VGA_HEIGHT/2 + VGA_HEIGHT/4));
	cvResize(gTempImage,gLeftImage,CV_INTER_LINEAR);
	cvSetImageROI(gTempImage,cvRect(VGA_WIDTH/2,VGA_HEIGHT/4,VGA_WIDTH  ,VGA_HEIGHT/2 + VGA_HEIGHT/4));
	cvResize(gTempImage,gRightImage,CV_INTER_LINEAR);

	cvSmooth(gLeftImage,gFliterImage,CV_MEDIAN,11,11,0,0);
	cvCvtColor(gFliterImage,gGrayLeftImage,CV_BGR2GRAY);
	cvSmooth(gRightImage,gFliterImage,CV_MEDIAN,11,11,0,0);
	cvCvtColor(gFliterImage,gGrayRightImage,CV_BGR2GRAY);

	cvFindStereoCorrespondence( gGrayLeftImage, gGrayRightImage, CV_DISPARITY_BIRCHFIELD, gGrayDepthImage, 127, 15, 3, 6, 8, 15 );  
	cvCvtColor(gGrayDepthImage,gDepthImage,CV_GRAY2BGR);
	cvScale(gDepthImage,gDepthImage,255/100);
            
#ifdef SHOW_WINDOW
	cvNamedWindow("Depth", CV_WINDOW_AUTOSIZE);
	cvShowImage("Depth", gDepthImage);
#endif //SHOW_WINDOW

}


bool getPosition(int diff, double* angle, double* distance)
{
    CvMoments moments;
    CvPoint center;
    bool presence=false;
    int count;
    char title[100];
    
    if(diff!=0){
		
		*distance = DIFFDIST/(double)diff;
        cvThreshold( gGrayDepthImage, gGrayThrDepthImage, diff, 255.0, CV_THRESH_BINARY );

    	cvMoments(gGrayThrDepthImage, &moments,0);
    	center.x = moments.m10/moments.m00;
    	center.y = moments.m01/moments.m00;
       	*angle = atan(((center.x  - diff*0.5 - VGA_WIDTH/4)/(((VGA_WIDTH/4)/((*distance)*0.5)))/(*distance)));

		cvCvtColor(gGrayThrDepthImage,gThrDepthImage,CV_GRAY2BGR);
    	count = cvCountNonZero(gGrayThrDepthImage);
    	if(count > COUNT_THR){
			presence = true;
	       	cvCircle( gThrDepthImage, center, 4, CV_RGB(255,0,0), 2, 4, 0);
    	}
#ifdef SHOW_WINDOW
		sprintf(title,"Diff %d, Distance %f",diff, *distance);
	    cvNamedWindow(title, CV_WINDOW_AUTOSIZE);
    	cvShowImage(title, gThrDepthImage);
#endif //SHOW_WINDOW
	}

	return presence;
}

int main()
{
    double distance = 0;
    double angle = 0;
    int diff;

    initCamera();

    smd_cmd(devh);

    gOrigImage = cvCreateImageHeader(cvSize(VGA_WIDTH, VGA_HEIGHT),IPL_DEPTH_8U,3);

    while(1){
       if(cbFlag == true){
            drawFlag = true;
            
            distance = 0;
            angle = 0;
            
            cvSetData(gOrigImage, gBGRFrame->data, VGA_WIDTH * 3);

#ifdef SHOW_WINDOW
            cvNamedWindow("Original", CV_WINDOW_AUTOSIZE);
            cvShowImage("Original", gOrigImage);
#endif //SHOW_WINDOW

			getDepthImage();
		
			for(diff = 10;diff <= 30; diff+= 10)
				if(getPosition(diff, &angle, &distance))

            printf("distance %f, angle %f \n", distance, angle*180/CV_PI);
                      
            drawFlag = false;
            cvWaitKey(1);
            cbFlag = false;
        }
    }
    
    uvc_stop_streaming(devh);
    puts("Done streaming.");
    uvc_close(devh);
    puts("Device closed");
    uvc_unref_device(dev);
    uvc_exit(ctx);
    puts("UVC exited");

    return 0;
}

