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

// #include "../inc/jetson_uart.h"

#include <pthread.h>

using namespace dlib;

#define _DEBUG_
// #define SHOW_FPS
// #define STREAM
// #define SAVE_VID

#define FRAME_WIDTH     320
#define FRAME_HEIGHT    240
// #define FRAME_WIDTH     640
// #define FRAME_HEIGHT    480
#define VID_SRC 1

// #define SEND_DATA
// #define RECV_DATA

DetectLane *lane_detect;
Control *car_control;
// SignDetect *sign_detect;
// JS_UART *js_uart;

Mat src;

const string Video_Path_Host = "/home/qthai/Downloads/Code_sua_xiu_full_dg_20_4/Vid_full_duong_bug_khuc_cua.avi";
const string Video_Path_Pi = "/home/jetson/Code_Thai/vid/Vid_right_ver3.avi";

uint8_t u8_Buf[100];
uint16_t u16_data_send[2];
uint8_t u8_save_txt_buf[100];

int flg = 2; //binh thuong, di thang
// volatile bool mat_lock = false;

Scalar minHSV_lft_rgt_sign = Scalar(78, 66, 0);
Scalar maxHSV_lft_rgt_sign = Scalar(132, 255, 255);

Scalar minHSV_stop_sign = Scalar(0,136,177);
Scalar maxHSV_stop_sign = Scalar(255,255,255);

int16_t volatile found_sign_main = 0;
// int16_t* p_found_sign_main = &found_sign_main;
Mat sign_crop;

// int16_t *param = &found_sign_main;
// *param = 0;

bool volatile mat_lock = false;

cv::Mat pre_process_svm(const cv::Mat &src);

cv::Mat pre_process_svm(const cv::Mat &src)
{
    Mat gray, src_hsv;
    Mat src_inrange_lft_rgt, src_inrange_stop;
    Mat src_crop;

    Mat src_nothing = Mat::zeros(Size(50,50), CV_8UC3);
    Rect roi;
    int x_rect=0, y_rect=0, width_r=0, height_r=0;
    cv::Point tl_rect = Point(-1, -1);
    cv::Point br_rect = Point(-1, -1);

    cvtColor(src, src_hsv, COLOR_BGR2HSV);

    // inRange(src_hsv, Scalar(minHSV[0], minHSV[1], minHSV[2]) , Scalar(maxHSV[0], maxHSV[1], maxHSV[2]), src_inrange);
    inRange(src_hsv, minHSV_lft_rgt_sign , maxHSV_lft_rgt_sign, src_inrange_lft_rgt);
    inRange(src_hsv, minHSV_stop_sign    , maxHSV_stop_sign   , src_inrange_stop);
    Mat mask_combine = src_inrange_lft_rgt + src_inrange_stop;

    std::vector<std::vector<cv::Point> > contours; //Tao 1 vector 2 chieu voi moi phan tu la Point
    std::vector<Vec4i> hierarchy;
    findContours(mask_combine, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
    std::vector<cv::Rect> boundRect( contours.size() );

    float max_rect_area = 0;
    size_t max_rect_ind = 0;

    // Mat drawing = Mat::zeros(mask_combine.size(), CV_8UC1);
    for (int i = 0; i < contours.size(); i++)
    {
        int area = contourArea(contours.at(i));
        // drawContours(drawing, contours, i, 255, 2, 8, hierarchy, 0, Point());

        if (area > 200) //loai bo contour nhieu
            boundRect[i] = boundingRect( Mat(contours[i]) );

        float tmp_rect_area = (boundRect[i].width)*(boundRect[i].height);
        if (tmp_rect_area > max_rect_area){ //lay contour co dien tich lon nhat
            if ( abs( boundRect[i].width - boundRect[i].height ) < 30){ //contour gan vuong
                max_rect_area = tmp_rect_area;
                max_rect_ind = i;
            }
        }
        // cv::rectangle( src, boundRect[i].tl(), boundRect[i].br(), Scalar(255,0,0), 2, 8, 0 );
    }
    // cv::rectangle(src, boundRect[max_rect_ind].tl(), boundRect[max_rect_ind].br(), Scalar(0,0,255), 4, 8, 0 );

    if (boundRect.size() > 0)
    {
        tl_rect = boundRect[max_rect_ind].tl();
        br_rect = boundRect[max_rect_ind].br();
        width_r  = br_rect.x - tl_rect.x + 10;
        height_r = br_rect.y - tl_rect.y + 10;

        x_rect = tl_rect.x - 10;
        y_rect = tl_rect.y - 10;
        // width_r = width_r + 20;
        // height_r = height_r + 20;

        if (y_rect < 0)
            y_rect = 0;
        if (width_r > 320)
            width_r = 320;
        if (height_r > 240)
            height_r = 240;
        roi = Rect(x_rect, y_rect,  width_r, height_r);
    }

	// Rect r = Rect(boundRect[max_rect_ind].width,  boundRect[max_rect_ind].height);
    // if ( boundRect.size() > 0){
        // src_crop = src(boundRect[max_rect_ind]).clone(); //crop vung co bien bao
        // if(roi.x >= 0 && roi.y >= 0 && roi.width + roi.x < FRAME_HEIGHT && roi.height + roi.y < FRAME_WIDTH)

    // }
    if(roi.x >= 0 && roi.y >= 0 && roi.width + roi.x < FRAME_HEIGHT && roi.height + roi.y < FRAME_WIDTH)
    {
        // while (mat_lock);
        // mat_lock = true;
        src_crop = src(roi).clone();
        // mat_lock = false;
    }
    if ( (src_crop.rows > 0) && (src_crop.cols > 0) ){
        // src_crop = src(roi).clone();
        found_sign_main = 1;
        return src(roi);
    }
    else{
        found_sign_main = 0;
    	return src_nothing;
    }
}

int cnt = 0;

void* get_blah_blah(void* arg )
{
    SignDetect *sign_detect;
    sign_detect = new SignDetect();
    // int16_t *p_found_flg = (int16_t *)arg;
    sign_detect->Load_SVM_signs_detector();
    while(1)
    {
        // int16_t p_found_flg = *((int16_t *)arg);

        // cout << "OK" << endl;
        // cout << *p_found_flg << endl;
        if ( found_sign_main == 1)
        {
            while(mat_lock);
            mat_lock = true;
            sign_detect->sign_detect_update(sign_crop);
            mat_lock = false;

            if((sign_detect->detect_index).size() == 0){
                cout << "Not found" << endl;
            }

            for (int i=0;i<(sign_detect->detect_index).size() ;i++)
            {
                if ( (sign_detect->detect_index)[i] == 1)//turn right ahead sign
                {
                    cout << "right" << endl;
                }
                else if ( (sign_detect->detect_index)[i] == -1)//turn left ahead sign
                {
                    cout << "left" << endl;
                }
                else if ( (sign_detect->detect_index)[i] == 0)//stop sign
                {
                    cout << "stop" << endl;
                }
            }

            // cout << "OK 1" <<endl;
            
        }
        
        
    }
    
}


int main(int argc, char **argv)
{
    bool playVideo = true;
#ifdef STREAM
    VideoCapture capture(Video_Path_Host);
#else
    VideoCapture capture(VID_SRC);
#endif

#ifdef SAVE_VID
    cv::VideoWriter video(Video_Path_Pi,CV_FOURCC('M','J','P','G'),30, Size(FRAME_WIDTH,FRAME_HEIGHT), true);
#endif

    lane_detect = new DetectLane();
    car_control = new Control();
    // sign_detect = new SignDetect();
    // js_uart = new JS_UART();
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

    // capture >> src;
    // src_cp = src.clone();
    // int16_t *param = (int16_t *)malloc(sizeof(int16_t));
    // *param = *p_found_sign_main;

    sign_crop = Mat::zeros(Size(50,50), CV_8UC3);
    // void *retval = malloc(sizeof(int));
	pthread_t thread;
	int rc;
	rc = pthread_create(&thread, NULL, get_blah_blah , NULL);
	cout << "Sign Detection Thread Created" << endl;

    if (rc) {
        cout << "Error:unable to create thread," << rc << endl;
        exit(-1);
    }
    
    // int ret_serial = js_uart->jetson_uart_init();

    while (true)
    {
        if (playVideo){
            capture >> src;
            // flip(src, src, 1);
        }

        if (src.empty()){
            break;
        }

        // while(mat_lock);
        // mat_lock = true;
        // src_cp = src.clone();
        // mat_lock = false;

        sign_crop = pre_process_svm(src);
        imshow("crop sign", sign_crop);
        // cout << "found_sign_main " << found_sign_main << endl;
        
#ifdef SAVE_VID
        video.write(src);
#endif
//** Image Processing Start Here **//
        lane_detect->Trackbar_Window();
        lane_detect->update(src);
//** Image Processing End Here  **//

//** Control Start Here         **//
        car_control->driverCar_ver2(lane_detect->center_lft_lane, lane_detect->center_rgt_lane, lane_detect->center_road_lane, flg);
        car_control->my_pub(u16_data_send);
        // cout << "[" << u16_data_send[0] << "," << u16_data_send[1]  << "]" << endl;        
//** Control End Here           **//

#ifdef SEND_DATA
        sprintf((char*)(u8_Buf), "[%d,%d]", u16_data_send[0], u16_data_send[1]);
        ssize_t ret_write = write(ret_serial, (char*)(u8_Buf), 8); //chi gui 8 byte
        // uart->uart_string_send(u8_Buf);
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

        char key = cv::waitKey(1)&0xFF;
        if(key == 'p')
            playVideo = !playVideo; 
        else if (key == 'q')
            break;
#endif
    }
    
    // clean_and_exit(0);
    
    capture.release();
	cv::destroyAllWindows();
	return 0;
}
