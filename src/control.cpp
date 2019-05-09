#include "../inc/control.h"
#include "../inc/pi_lane_detect.h"

// #define DRAW
// #define PRINT_DEBUG

Control::Control()
{
	carPos.x = DetectLane::BIRDVIEW_WIDTH/2; //120
	carPos.y = DetectLane::BIRDVIEW_HEIGHT - 40; //can calib lai: 280
    pre_angle = 0.0;
    current_angle = 0.0;
}
Control::~Control(){}

int8_t Control::pid_init(void)
{
    // Kp lon = 0.4; Kp nho: 0.35 la thong so chay duoc
    // angle_crtl_pid.Kp = 0.4;
    angle_crtl_pid.Kp = 0.35;
    angle_crtl_pid.Ki = 0.0012;
    angle_crtl_pid.Kd = 0.0;
    angle_crtl_pid.Ts = 0.01;
    angle_crtl_pid.PID_Saturation = 25;

    // angle_small_ctrl_pid.Kp = 0.35;
    angle_small_ctrl_pid.Kp = 0.3;
    angle_small_ctrl_pid.Ki = 0.0012;
    angle_small_ctrl_pid.Kd = 0.0;
    angle_small_ctrl_pid.Ts = 0.01;
    angle_small_ctrl_pid.PID_Saturation = 10;
    return 1;
}

uint16_t Control::Convert_to_DutyCycles(float udk, float angle){
//** Servo duty limit:    55    75    95        **//
//** Angle limit:       -25.0   0.0   25.0      **//
    int duty_cyc;
    
    if ( (angle > -10.0 ) && (angle < 10.0) )
	   duty_cyc = (int) (GAIN_CVT_SMALL*(-udk) + BIAS_CVT_SMALL);
    else
        duty_cyc = (int) (GAIN_CVT_BIG*(-udk) + BIAS_CVT_BIG);
    return duty_cyc;
}

float Control::compute_angle_error(const Point &dst)
{
    float angle;
    float dx, dy; //phai de dang float (khong de dang int) vi chia se ra 0
    
    if (dst.x == carPos.x){
        // cout << "carpos = x" << endl;
        return 0; // tren mot duong thang doc di dung huong
    }
    // if (dst.y == carPos.y){
    //     cout << "carpos nam ngang" << endl;
    //     return (dst.x < carPos.x ? -90 : 90); // tren duong thang ngang(horizon) queo 90
    // }
	
    // if (abs(curr_carPos_x - pre_carPos_x) < 0.3*curr_carPos_x){
    dx = dst.x - carPos.x;
	if (dx > 120)
	    dx = 120;
	if (dx < -120)
	    dx = -120;
    // }
    dy = carPos.y - dst.y;
    angle = atan(dx/dy) * 180 / PI;
    // angle_error = current_angle - pre_angle;
    // if (dx < 0) //center lane lech trai so voi carPos (phai di chech ve ben trai)
    // {
    //     angle_error = -atan(-dx / dy) * 180 / PI;
    // }
    // else //center lane lech phai so voi carPos (phai di chech ve ben phai)
    // {
    //     angle_error = atan(dx / dy) * 180 / PI;
    // }
    return angle;
}

float Control::pid_process_raw(PID_PARAMETERS* pid_parameter,double current_error)
{
	static double P_part, I_part, D_part;
//	Update Parameters
	pid_parameter->error__ = pid_parameter->error_;
	pid_parameter->error_ = pid_parameter->error;
	pid_parameter->error = current_error;
	pid_parameter->u_ = pid_parameter->u;
	
	// P_part = pid_parameter->Kp * (pid_parameter->error - pid_parameter->error_);
    P_part = pid_parameter->Kp * (pid_parameter->error);
    I_part = pid_parameter->Ki * pid_parameter->Ts * pid_parameter->error;
    D_part = (pid_parameter->Kd / pid_parameter->Ts) * (pid_parameter->error - (2 * pid_parameter->error_) + pid_parameter->error__);
    pid_parameter->u = P_part + I_part + D_part;
	
	//pid_parameter->u = pid_parameter->u_ + pid_parameter->Kp * (pid_parameter->error - pid_parameter->error_)
	//		+ pid_parameter->Ki * pid_parameter->Ts * pid_parameter->error
	//		+ (pid_parameter->Kd / pid_parameter->Ts) * (pid_parameter->error - (2 * pid_parameter->error_) + pid_parameter->error__);

	if (pid_parameter->u > pid_parameter->PID_Saturation)
	{
		pid_parameter->u = pid_parameter->PID_Saturation;
	}
	else if (pid_parameter->u < (-pid_parameter->PID_Saturation))
	{
		pid_parameter->u = -pid_parameter->PID_Saturation;
	}
	// cout << "udk: " << pid_parameter->u << endl;

	return pid_parameter->u;
}

int8_t Control::pid_set_para(PID_PARAMETERS* pid_parameter,float Kp,float Ki, float Kd)
{
   	pid_parameter->Kp = Kp;
	pid_parameter->Ki = Ki;
	pid_parameter->Kd = Kd;
	return 1;
}

void Control::driverCar_ver2(const Point &center_left, const Point &center_right, const Point &center_lane, int flag)
{
    cv::Point null  = cv::Point(-1,-1);
    float cur_angle, udk;
    uint16_t duty = 75;
    // Scalar(x,y,z) : Blue Green Red
    if (flag == 2) // binh thuong
    {
        if (center_left != null && center_right != null) // co ca 2 lane
        {
            // cur_angle = compute_angle_error( (center_left + center_right)/2 );
            cur_angle = compute_angle_error( center_lane);
            if ( (cur_angle > -10.0) && (cur_angle < 10.0) ){
                udk = pid_process_raw(&angle_small_ctrl_pid, cur_angle);
            }
            else{
                udk = pid_process_raw(&angle_crtl_pid, cur_angle);
            }
        }

        else if ((center_left != null) && (center_right == null) )// co lane trai
        {
            // cur_angle = compute_angle_error( Point(center_left.x + laneWidth/2, center_left.y) ); //offset nua khoang cach lane duong
            cur_angle = compute_angle_error(center_lane);
            if ( (cur_angle > -10.0) && (cur_angle <10.0) ){
                udk = pid_process_raw(&angle_small_ctrl_pid, cur_angle);
            }
            else{
                udk = pid_process_raw(&angle_crtl_pid, cur_angle);
            }
        }

        else if ( (center_right != null) && (center_left == null) )// co lane phai
        {
            // cur_angle = compute_angle_error( Point(center_right.x - laneWidth/2, center_right.y) );
            cur_angle = compute_angle_error(center_lane);
            if ( (cur_angle > -10.0) && (cur_angle <10.0) ){
                udk = pid_process_raw(&angle_small_ctrl_pid, cur_angle);
            }
            else{
                udk = pid_process_raw(&angle_crtl_pid, cur_angle);
            }
        }
    }

    // else if (flag == 0) // re trai
    // {
    //     while (left[pnt_target_ptr] == DetectLane::null) // ko co lane
    //     {
    //         pnt_target_ptr--;
    //         if (pnt_target_ptr < 0)
    //             return;
    //     }
    //     error = pid_process(left[pnt_target_ptr] + Point(laneWidth / 2, 0));

    //     // ve diem bam theo
    //     circle(lane, left[pnt_target_ptr], 1, Scalar(0, 0, 255), 2, 8, 0);
        
    // }
    
    // else if (flag == 1) // re phai
    // {
    //     while (right[pnt_target_ptr] == DetectLane::null) // ko co lane
    //     {
    //         pnt_target_ptr--;
    //         if (pnt_target_ptr < 0)
    //             return;
    //     }
    //     error = pid_process(right[pnt_target_ptr] - Point(laneWidth / 2, 0));
    //     // ve diem bam theo
    //     circle(lane, right[pnt_target_ptr], 1, Scalar(255, 0, 0), 2, 8, 0);
        
    // }

    else
    {
        cout << "Something not right, check your code again" << endl;
    }
    duty = Convert_to_DutyCycles(udk, cur_angle);
#ifdef PRINT_DEBUG
    // cout <<  "," << current_angle << "," << pre_angle << "," << angle_error << endl;
    // cout << angle_error << endl;
    cout << "angle_error: " << cur_angle << " ," << udk << " ," << duty << endl;
#endif
    *p_data = duty;
    *(p_data + 1) = 40;
}

void Control::my_pub(uint16_t* data_buf)
{
    data_buf[0] = Angle_Velo_data[0];
    data_buf[1] = Angle_Velo_data[1];
}

// void Control::driverCar(const vector<Point> &left, const vector<Point> &right, float velocity, int flag)
// {
//     int pnt_target_ptr = left.size() - 16; //pnt_target_ptr = 32-16=16
//     float angle_error, udk;
//     Mat lane = Mat::zeros(Size(DetectLane::BIRDVIEW_WIDTH, DetectLane::BIRDVIEW_HEIGHT), CV_8UC3);
//     static Point null = Point();
//     uint16_t duty = 75;
    
//     if (flag == 2) // binh thuong
//     {
//         while (left[pnt_target_ptr] == DetectLane::null && right[pnt_target_ptr] == DetectLane::null) // ko co lane
//         {
//             pnt_target_ptr--;
//             if (pnt_target_ptr < 0)
//                 return;
//         }
//         if (left[pnt_target_ptr] != DetectLane::null && right[pnt_target_ptr] != DetectLane::null) // co ca 2 lane
//         {
//      // Scalar(x,y,z) : Blue Green Red
//         angle_error = compute_angle_error((left[pnt_target_ptr] + right[pnt_target_ptr]) / 2);
//         if ( (angle_error > -10.0) && (angle_error <10.0) )
//             udk = pid_process_raw(&angle_small_ctrl_pid, angle_error);
//         else
//             udk = pid_process_raw(&angle_crtl_pid, angle_error);

//      #ifdef DRAW
//             circle(lane, left[pnt_target_ptr], 1, Scalar(0, 0, 255), 2, 8, 0); //Mau do lane trai
//             circle(lane, right[pnt_target_ptr], 1, Scalar(255, 0, 0), 2, 8, 0); //Mau xanh duong lane phai
//      #endif
//         }
//         else if (left[pnt_target_ptr] != DetectLane::null) // co lane trai
//         {
//             angle_error = compute_angle_error(left[pnt_target_ptr] + Point(laneWidth / 2, 0));
//             // angle_error = compute_angle_error(left[pnt_target_ptr]);
//      if ( (angle_error > -10.0) && (angle_error <10.0) )
//          udk = pid_process_raw(&angle_small_ctrl_pid, angle_error);
//      else
//      udk = pid_process_raw(&angle_crtl_pid, angle_error);
//      #ifdef DRAW
//             circle(lane, left[pnt_target_ptr], 1, Scalar(0, 0, 255), 2, 8, 0);
//      #endif
//         }
//         else // co lane phai
//         {
//             angle_error = compute_angle_error(right[pnt_target_ptr] - Point(laneWidth / 2, 0));
//             // angle_error = compute_angle_error(right[pnt_target_ptr]);
//      if ( (angle_error > -10.0) && (angle_error <10.0) )
//      udk = pid_process_raw(&angle_small_ctrl_pid, angle_error);
//      else
//      udk = pid_process_raw(&angle_crtl_pid, angle_error);
//         #ifdef DRAW
//             circle(lane, right[pnt_target_ptr], 1, Scalar(255, 0, 0), 2, 8, 0);
//      #endif
//         }
//     }

//     else
//  {
//      cout << "Something not right, check your code again" << endl;
//  }
//     duty = Convert_to_DutyCycles(udk);
//     // cout << angle_error << ", udk: " << udk;
//     // cout << " duty: " << duty << endl;
//     #ifdef DRAW
//     circle(lane, carPos, 1, Scalar(255, 255, 255), 2, 8, 0); //khong biet lane trai hay phai: mau trang
//     imshow("Debug", lane);`
//     #endif

//     /********************************************************************/
//     *p_data = duty;
//     *(p_data + 1) = 40;
//     /********************************************************************/
// }
