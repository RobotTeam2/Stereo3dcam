#include <stdio.h>
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

//#define SHOW_WINDOW
#define COUNT_THR (768*5) //5% of 320x240 
using namespace std;

void smd_cmd(uvc_device_handle_t *devh);

IplImage* cvImg= cvCreateImage(cvSize(640,480),8,3);;
uvc_frame_t *bgr = uvc_allocate_frame(640 * 480 * 3);
IplImage* leftim = cvCreateImage(cvSize(320,240),8,3);
IplImage* rightim = cvCreateImage(cvSize(320,240),8,3);
IplImage* tmp = NULL;
IplImage* filter = cvCreateImage(cvGetSize(leftim),8,3);

IplImage* l = cvCreateImage(cvGetSize(leftim),8,1);
IplImage* r = cvCreateImage(cvGetSize(leftim),8,1);

IplImage* depth = cvCreateImage(cvGetSize(l),8,1);
IplImage* dest = cvCreateImage(cvGetSize(l),8,3);

IplImage* depth10 = cvCreateImage(cvGetSize(l),8,1);
IplImage* depth20 = cvCreateImage(cvGetSize(l),8,1);
IplImage* depth30 = cvCreateImage(cvGetSize(l),8,1);

bool drawFlag=false;
bool cbFlag=false;
  
void cb(uvc_frame_t *frame, void *ptr)
{

    uvc_error_t ret=(uvc_error_t)0;

    cbFlag=true;
//  printf("callback! length = %u, ptr = %d\n", (unsigned int)frame->data_bytes, (long)ptr);
  
    if(drawFlag == true) return;

    ret = uvc_any2bgr(frame, bgr);
    if(ret)uvc_perror(ret, "uvc_any2bgr");

}

int initCamera()
{
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

    res = uvc_get_stream_ctrl_format_size(devh, &ctrl,UVC_FRAME_FORMAT_YUYV,640, 480, 30);
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

int getDepth(
)
{

            tmp=cvCloneImage(cvImg);
            cvSetImageROI(tmp,cvRect(0,120,320,360));
            cvResize(tmp,leftim,CV_INTER_LINEAR);
            cvSetImageROI(tmp,cvRect(320,120,320,360));
            cvResize(tmp,rightim,CV_INTER_LINEAR);
            cvReleaseImage(&tmp);

            cvSmooth(leftim,filter,CV_MEDIAN,11,11,0,0);
            cvCvtColor(filter,l,CV_BGR2GRAY);

            cvSmooth(rightim,filter,CV_MEDIAN,11,11,0,0);
            cvCvtColor(filter,r,CV_BGR2GRAY);
            
            cvFindStereoCorrespondence( l, r, CV_DISPARITY_BIRCHFIELD, depth, 127, 15, 3, 6, 8, 15 );  

            cvCvtColor(depth,dest,CV_GRAY2BGR);
            cvScale(dest,dest,255/100);
            
#ifdef SHOW_WINDOW
            cvNamedWindow("Depth", CV_WINDOW_AUTOSIZE);
            cvShowImage("Depth", dest);
#endif //SHOW_WINDOW
	
}


int getPosition( // 名前ももうちょっと考えたい
// -- 引数を考える
// 画像のポインタ(depth画像)
// 取得したいdepth
// ディスプレイ表示の有無
)
{
    CvMoments moments10;
    CvMoments moments20;
    CvMoments moments30;
    CvPoint center10;
    CvPoint center20;
    CvPoint center30;
    
// 固定値で入れているものをパラメータ化する
            
            cvThreshold( depth, depth10, 10.0, 255.0, CV_THRESH_BINARY );
            cvThreshold( depth, depth20, 20.0, 255.0, CV_THRESH_BINARY );
            cvThreshold( depth, depth30, 30.0, 255.0, CV_THRESH_BINARY );


            cvMoments(depth10, &moments10,0);
            center10.x = moments10.m10/moments10.m00;
            center10.y = moments10.m01/moments10.m00;

            cvCvtColor(depth10,dest,CV_GRAY2BGR);
            count = cvCountNonZero(depth10);
//            printf("depth10:%d \n",count);
            if(count > COUNT_THR){
                cvCircle( dest, center10, 4, CV_RGB(255,0,0), 2, 4, 0);
                distance = 90.0;
                angle = atan((((center10.x - 5) - 160)/(160/45))/distance);
            }

// 20と30でちがうのはここだけ
                angle = atan((((center20.x - 10) - 160)/(160/22.5))/distance);
                angle = atan((((center30.x - 15) - 160)/(160/15))/distance);



#ifdef SHOW_WINDOW
            cvNamedWindow("Depth10", CV_WINDOW_AUTOSIZE);
            cvShowImage("Depth10", dest);
#endif //SHOW_WINDOW
}

int main()
{
    uvc_context_t *ctx=NULL;
    uvc_device_t *dev=NULL;
    uvc_device_handle_t *devh=NULL;
    uvc_stream_ctrl_t ctrl;
    uvc_error_t res=(uvc_error_t)0;

    
    double distance = 0;
    double angle = 0;

    int count;

//-- cameraの初期化


//-- 特殊モード突入
    smd_cmd(devh);

//-- メインループ
    while(1){
       if(cbFlag == true){
            drawFlag = true;
            
            distance = 0;
            angle = 0;
            
            cvImg = cvCreateImageHeader(cvSize(bgr->width, bgr->height),IPL_DEPTH_8U,3);

            cvSetData(cvImg, bgr->data, bgr->width * 3);

#ifdef SHOW_WINDOW
            cvNamedWindow("Original", CV_WINDOW_AUTOSIZE);
            cvShowImage("Original", cvImg);
#endif //SHOW_WINDOW

//-- depth画像取得

//-- depth10の処理 --

//-- depth20の処理 --

//-- depth30の処理 --

            printf("distance %f, angle %f \n", distance, angle*180/CV_PI);
                      
            cvReleaseImageHeader(&cvImg);
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

