#include <dlib/svm_threaded.h>
#include <dlib/string.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_processing.h>
#include <dlib/data_io.h>
#include <dlib/image_transforms.h>
#include <dlib/cmd_line_parser.h>

#include <dlib/opencv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <time.h>
#include <iostream>
#include <fstream>
#include <cstdlib>

using namespace std;
using namespace dlib;
using namespace cv;

#define _DEBUG_
#define FRAME_WIDTH     320
#define FRAME_HEIGHT    240
#define VID_SRC 1

RNG rng(12345);

struct TrafficSign {
  string name;
  string svm_path;
  rgb_pixel color;
  TrafficSign(string name, string svm_path, rgb_pixel color) :
    name(name), svm_path(svm_path), color(color) {};
};

Mat src;

int main(int argc, char** argv) {

	bool detect_flg = false; //Flag nhan biet co detect duoc bien bao hay khong
	VideoCapture capture(VID_SRC);

	typedef scan_fhog_pyramid<pyramid_down<6> > image_scanner_type; 
    // Get the upsample option from the user but use 0 if it wasn't given.

	// Load SVM detectors
	cout << "Loading SVM detectors..." << endl;
	std::vector<TrafficSign> signs;

	signs.push_back(TrafficSign("right", "../svm_detectors/turn_right_ver2.svm", rgb_pixel(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255)) ));
	signs.push_back(TrafficSign("left" , "../svm_detectors/turn_left_ver1.svm",  rgb_pixel(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255)) ));
    // signs.push_back(TrafficSign("stop", "../resources/detectors/stop-detector.svm", rgb_pixel(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255)) ));
	
	std::vector<object_detector<image_scanner_type> > detectors;

	for (int i = 0; i < signs.size(); i++) {
		object_detector<image_scanner_type> detector;
		deserialize(signs[i].svm_path) >> detector;
		detectors.push_back(detector);
	}
	cout <<"Finish loaded SVM detectors" <<endl;
			
	while (true)
	{
		if( !capture.isOpened() )
        	throw "Error when reading steam_avi";
        
	    capture >> src;

	    if (src.empty()){
	        break;
	    }

	    // cout << "detector size: " << detectors.size() << endl;

	    cv::Rect draw_box;
	    std::vector <dlib::rect_detection> rects;

	    Mat src_copy = src.clone();

	    dlib::cv_image<dlib::bgr_pixel> images_HOG(src_copy);
	    rects.clear();
	    draw_box.empty();
	    detect_flg = true;
	    dlib::evaluate_detectors(detectors, images_HOG, rects);
	    for (int i=0;i<rects.size();i++)
	    {
	    	cout << "confidence: " << rects[i].detection_confidence <<endl;
	    	if (rects[i].detection_confidence < 0.1)
	    	{
	    		detect_flg = false;
	    	}

	    	if (detect_flg == true)
	    	{
			    draw_box.x=rects[i].rect.left();
		        draw_box.y=rects[i].rect.top();
		        draw_box.width = rects[i].rect.right()-draw_box.x;
		        draw_box.height = rects[i].rect.bottom()-draw_box.y;
		        cv::rectangle(src,draw_box.tl() , draw_box.br(), Scalar(0,0,255), 2, 8, 0);

				// cv::rectangle( img, boundRect[i].tl(), boundRect[i].br(), Scalar(0,0,255), 2, 8, 0 );

		        cv::putText(src, signs[rects[i].weight_index].name , draw_box.tl() , FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);
		        // putText(src, "right", Point(25,25), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);
		        cout<< signs[rects[i].weight_index].name<<endl;
	    	}
		}

		// putText(test_left_rgt, Utility().intToString(i), lane1[i], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);	

        #ifdef _DEBUG_
			cv::imshow("View", src);
		#endif

        char key = cv::waitKey(1)&0xFF;
        if (key == 'q')
            break;
	}

	cv::destroyAllWindows();
	return 0;
}

