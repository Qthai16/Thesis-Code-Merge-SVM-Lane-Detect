#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <math.h>
#include <time.h>

#include "../inc/pi_lane_detect.h"
#include "../inc/control.h"
#include "../inc/utility.h"
#include "../svm_sign_detect/src/sign_detect.h"

#include <cmath> //openmp, thuc hien multi processing

// #include "../inc/raspi_uart.h"

#include <pthread.h>

using namespace dlib;

#define _DEBUG_
// #define SHOW_FPS
// #define STREAM
// #define SAVE_VID

#define FRAME_WIDTH     320
#define FRAME_HEIGHT    240
#define VID_SRC 1

// #define SEND_DATA
// #define RECV_DATA

DetectLane *lane_detect;
Control *car_control;
// SignDetect *sign_detect;
// PI_UART *uart;

Mat src;
Mat src_cp;

const string Video_Path_Host = "/home/qthai/Downloads/Code_sua_xiu_full_dg_20_4/Vid_full_duong_bug_khuc_cua.avi";
const string Video_Path_2 = "/home/pi/Code_new/Code_sua_chut_xiu_17_4/Code_no_windows_16_4/Vid_full_duong_4.avi";

uint8_t u8_Buf[100];
uint16_t u16_data_send[2];
uint8_t u8_save_txt_buf[100];

int flg = 2; //binh thuong, di thang
volatile bool mat_lock = false; 

/**void* get_blah_blah(void* arg)
{
    SignDetect *sign_detect;
    sign_detect = new SignDetect();

    sign_detect->Load_SVM_signs_detector();
    while(1){
        cout << "OK" << endl;
        sign_detect->sign_detect_update(src_cp, mat_lock);
        for (int i=0;i<(sign_detect->detect_index).size() ;i++)
        {
            if ( (sign_detect->detect_index)[i] == 1)//turn right ahead
            {
                cout << "right" << endl;
            }
            else if ( (sign_detect->detect_index)[i] == -1)
            {
                cout << "left" << endl;
            }
            else if ( (sign_detect->detect_index)[i] == 0)
            {
                cout << "stop" << endl;
            }
        }
    }
    
}**/


int main(int argc, char **argv)
{
    bool playVideo = true;
#ifdef STREAM
    VideoCapture capture(Video_Path_Host);
#else
    VideoCapture capture(VID_SRC);
#endif

#ifdef SAVE_VID
    cv::VideoWriter video(Video_Path_2,CV_FOURCC('M','J','P','G'),30, Size(FRAME_WIDTH,FRAME_HEIGHT), true);
#endif

    lane_detect = new DetectLane();
    car_control = new Control();
    // sign_detect = new SignDetect();
    // uart = new PI_UART();
	if( !capture.isOpened() )
        throw "Error when reading steam_avi";
        
    // capture.set(CV_CAP_PROP_FPS, 30);
    capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

#ifdef SHOW_FPS
    int num_frames = 0;
    time_t start, end;
    time(&start);
    Utility::initTime();
#endif

    car_control->pid_init();
    // sign_detect->Load_SVM_signs_detector();


    capture >> src;
    src_cp = src.clone();
    
    pthread_t thread;
    int rc;
    cout << "main() : creating thread" << endl;
	// rc = pthread_create(&thread, NULL, get_blah_blah, NULL);

    // if (rc) {
    //     cout << "Error:unable to create thread," << rc << endl;
    //     exit(-1);
    // }

    while (true)
    {
        if (playVideo){
            capture >> src;
            // flip(src, src, 1);
        }

        if (src.empty()){
            break;
        }

        while(mat_lock);
        mat_lock = true;
        src_cp = src.clone();
        mat_lock = false;
        
#ifdef SAVE_VID
        video.write(src_cp);
#endif
//** Image Processing Start Here **//

// #pragma omp parallel
        // lane_detect->Trackbar_Window();
        lane_detect->update(src_cp);
//** Image Processing End Here  **//

//** Control Start Here         **//
        car_control->driverCar_ver2(lane_detect->center_lft_lane, lane_detect->center_rgt_lane, lane_detect->center_road_lane, flg);
        car_control->my_pub(u16_data_send);
        cout << "[" << u16_data_send[0] << "," << u16_data_send[1]  << "]" << endl;        
//** Control End Here           **//

#ifdef SEND_DATA
        sprintf((char*)(u8_Buf), "[%d,%d]", u16_data_send[0], u16_data_send[1]);
        uart->uart_string_send(u8_Buf);
#endif

#ifdef RECV_DATA
        uart->uart_string_recv(u8_Buf);
#endif

#ifdef SHOW_FPS
        ++num_frames;
        time(&end);
        double seconds = difftime(end, start);
        if (seconds >= 1.0)
        {
           cout << "FPS: " << num_frames << " Time: " << seconds << endl;
           sprintf((char*)(u8_save_txt_buf), "FPS: %d, Time: %lf\n", num_frames, seconds);
           Utility().appendLineToFile(u8_save_txt_buf);
           time(&start);
           num_frames = 0; 
        }
#endif



#ifdef _DEBUG_
        cv::imshow("View", src);

        char key = cv::waitKey(45)&0xFF;
        if(key == 'p')
            playVideo = !playVideo; 
        else if (key == 'q')
            break;
#endif
    }
    capture.release();
	cv::destroyAllWindows();
	return 0;
}
