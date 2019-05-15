#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <time.h>
#include <iostream>
#include <fstream>
#include <cstdlib>

#include "sign_detect.h"

using namespace std;
using namespace dlib;
using namespace cv;

#define VID_SRC 1

SignDetect *detect;

int main(int argc, char** argv) {
	VideoCapture capture(VID_SRC);
	Mat src;
	detect = new SignDetect();
	
	detect->Load_SVM_signs_detector();

	while (true)
	{
		if( !capture.isOpened() )
        	throw "Error when reading steam_avi";
        
	    capture >> src;

	    if (src.empty()){
	        break;
	    }

	    detect->sign_detect_update(src);
	    for (int i=0;i<(detect->detect_index).size() ;i++)
	    {
	    	if ( (detect->detect_index)[i] == 1)//turn right ahead
		    {
		    	cout << "right" << endl;
		    }
		    else if ( (detect->detect_index)[i] == -1)
		    {
		    	cout << "left" << endl;
		    }
		    else if ( (detect->detect_index)[i] == 0)
		    {
		    	cout << "stop" << endl;
		    }
	    }
	    
        char key = cv::waitKey(1)&0xFF;
        if (key == 'q')
            break;
	}
	cv::destroyAllWindows();
	return 0;
}

