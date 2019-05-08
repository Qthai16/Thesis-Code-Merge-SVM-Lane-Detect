#include "sign_detect.h"

SignDetect::SignDetect()
{

}
SignDetect::~SignDetect()
{

}

std::vector<TrafficSign> SignDetect::signs_data;
std::vector<object_detector<image_scanner_type> > SignDetect::detectors_data;
std::vector<int16_t> SignDetect::detect_index;

void SignDetect::Load_SVM_signs_detector(void) //** Load SVM detectors **//
{
	
	object_detector<image_scanner_type> tmp_detector;

	cout << "Loading SVM detectors..." << endl;

	signs_data.push_back( TrafficSign ("right", "../svm_sign_detect/svm_detectors/turn_right_ver2.svm", rgb_pixel(255, 0, 0), 1  ));
	signs_data.push_back( TrafficSign ("left" , "../svm_sign_detect/svm_detectors/turn_left_ver1.svm" , rgb_pixel(0, 0, 255), -1 ));
    signs_data.push_back( TrafficSign ("stop" , "../svm_sign_detect/svm_detectors/stop_ver1.svm"      , rgb_pixel(0, 255, 0), 0  ));
	
	for (int i = 0; i < signs_data.size(); i++) {
		deserialize(signs_data[i].svm_path) >> tmp_detector;
		detectors_data.push_back(tmp_detector);
	}
	// cout << "detector size: " << detectors.size() << endl;
	cout << "Finish loaded SVM detectors" << endl;
}

void SignDetect::sign_detect_update(const cv::Mat &src)
{
    Mat src_copy = src.clone();
    // bool detect_flg = true;

    // cv::Rect draw_box;
    std::vector <dlib::rect_detection> rects;

    dlib::cv_image<dlib::bgr_pixel> images_HOG(src_copy);
    // rects.clear();
    // draw_box.empty();
    detect_index.clear();
    evaluate_detectors(detectors_data, images_HOG, rects);

    for (int i=0;i<rects.size();i++)
    {
    	// cout << "confidence: " << rects[i].detection_confidence << endl;

    	// if (rects[i].detection_confidence < 0.04)
    	// {
    	// 	detect_flg = false;
    	// 	// cout << "detect flg false roi ne" << endl;
    	// }

    	// if (detect_flg == true)
    	// {
    		// // cout << "detect flg true roi ne" << endl;
		    // draw_box.x=rects[i].rect.left();
	     //    draw_box.y=rects[i].rect.top();
	     //    draw_box.width = rects[i].rect.right()-draw_box.x;
	     //    draw_box.height = rects[i].rect.bottom()-draw_box.y;
	     //    cv::rectangle(src,draw_box.tl() , draw_box.br(), Scalar(0,0,255), 2, 8, 0);
	        // cv::putText(src, signs_data[rects[i].weight_index].name , draw_box.tl() , FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);
	        // putText(src, "right", Point(25,25), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);
	        // cout<< signs_data[rects[i].weight_index].name<<endl;
	        // detect_ind_array[i] = signs_data[rects[i].weight_index].sign_index;
	        detect_index.push_back(signs_data[rects[i].weight_index].sign_index);
    	// }
	}

#ifdef SHOW_DETECT
	cv::imshow("Detect signs", src);
#endif
}

