#include "judge_gui.h"
#include <iostream>
#include <ros/node_handle.h>
#include <cstdlib>
#include "geometry_msgs/Vector3.h"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <algorithm>
namespace JudgeSystem
{
JudgeGui::JudgeGui(ros::NodeHandle& nh, std::string &path):_nh(nh){
    // ros::NodeHandle nh
    // _nh = nh;
    ros::start();
    _res_path = path;
    ros::Time::init();
    _is_start = false;
    _is_game_over = false;
    _A_win = false;
    _B_win = false;
    _home_a_blood_remain = 100.0;
    _home_b_blood_remain = 100.0;
    _blood_a_body_remain = 0.0;
    _blood_b_body_remain = 0.0;
    _A_color = cv::Scalar(0, 0, 255);
    _B_color = cv::Scalar(255, 0, 0);
    _font_color = cv::Scalar(0,0,0);
    _time_game_last = 180.0;
    set_ros_communication();
    
}

JudgeGui::~JudgeGui(){}

void JudgeGui::set_ros_communication(){
    _sub_models_pose_A = _nh.subscribe("/A/car_uav_pose", 1, 
            &JudgeGui::models_state_callback_a, this);
    _sub_models_pose_B = _nh.subscribe("/B/car_uav_pose", 1,
            &JudgeGui::models_state_callback_b, this);
    _sub_alive_model_A = _nh.subscribe("/A/alive_model", 1, 
        &JudgeGui::alive_model_callback_a, this);
    _sub_alive_model_B = _nh.subscribe("/B/alive_model", 1, 
        &JudgeGui::alive_model_callback_b, this);
    _sub_game_start = _nh.subscribe("/game_start_status", 1,
        &JudgeGui::game_state_callback, this);
    _pub_game_over = _nh.advertise<std_msgs::Bool>("/game_over_status", 1);
}

void JudgeGui::models_state_callback_a(const gazebo_msgs::ModelStatesConstPtr& msg){
    if (msg->name.empty()){
        return ;
    }
    auto name_list = msg->name;
    auto pose_list = msg->pose;
    auto twist_list = msg->twist;
    int count =  name_list.size();
    _model_pos_A.clear();
    for (int i = 0; i < count; i++){
        std::string name_tmp = name_list[i];    
        _model_pos_A[name_tmp] = pose_list[i].position;
 
    }
}

void JudgeGui::models_state_callback_b(const gazebo_msgs::ModelStatesConstPtr& msg){
    if (msg->name.empty()){
        return ;
    }
    auto name_list = msg->name;
    auto pose_list = msg->pose;
    auto twist_list = msg->twist;
    int count =  name_list.size();
    _model_pos_B.clear();
    for (int i = 0; i < count; i++){
        std::string name_tmp = name_list[i];    
        _model_pos_B[name_tmp] = pose_list[i].position;
    }
}

void JudgeGui::alive_model_callback_a(const judge_system_msgs::Alive_models& msg){
    _alive_models_A = msg;
}

void JudgeGui::alive_model_callback_b(const judge_system_msgs::Alive_models& msg){
    _alive_models_B = msg;
}

void JudgeGui::game_state_callback(const std_msgs::BoolConstPtr& msg){
    if (!_is_start){
        if (msg->data){
        _is_start = true;
        _start_time = ros::Time::now();
        }
    }
}

void JudgeGui::plot_result(){
    int width = 1000;
    int height = 1000;
    int width_new = 1600;
    int height_new = 1300;
    cv::Mat image(width_new, height, CV_8UC3, cv::Scalar(255,255,255));
    cv::Point origin(width/2, height/2 + 300);
    cv::circle(image, origin, 2, _font_color, 2);
    
    for (int i = -3; i < 4; i++){
        int line_x = width/2 + i*100;
        cv::line(image, cv::Point(line_x, height/2 - 400 + 300), cv::Point(line_x, height/2 + 400 + 300), cv::Scalar(100,100,100), 2);
    }
    for (int j = -4; j < 5; j++){
        int line_y = height/2 + j*100;
        cv::line(image, cv::Point(width/2 - 300, line_y + 300), cv::Point(width/2 + 300, line_y + 300), cv::Scalar(100,100,100), 2);
    }

    int count_a = _model_pos_A.size();
    int count_b = _model_pos_B.size();

    for (auto& pos_ : _model_pos_A){
        auto py = (int)(pos_.second.x * 10 + width/2 + 300);
        auto px = (int)(pos_.second.y * 10 + height/2);
        if ((px >= 0 && px < width) && (py >= 300 && py < height_new)){
            std::string name_str = pos_.first;
            cv::MarkerTypes type;
            if (name_str.find("car") != std::string::npos){
                type = cv::MARKER_TRIANGLE_UP;
                cv::drawMarker(image, cv::Point(px, py), _A_color, type, 20, 3);
                // name_str = name_str.erase(0, 3);
                cv::putText(image, name_str, cv::Point(px, py+20), cv::FONT_HERSHEY_SIMPLEX, 0.5, _font_color, 2);
            }else if(name_str.find("uav") != std::string::npos){
                type = cv::MARKER_STAR;
                cv::drawMarker(image, cv::Point(px, py), _A_color, type, 20, 3);
                // name_str = name_str.erase(0, 3);
                cv::putText(image, name_str, cv::Point(px, py+20), cv::FONT_HERSHEY_SIMPLEX, 0.5, _font_color, 2);
        
            }else{
                type = cv::MARKER_SQUARE;
                cv::rectangle(image,cv::Point(px-20, py-20), cv::Point(px+20, py+20) ,_A_color,3,8,0);
                cv::putText(image, "Home", cv::Point(px, py+20), cv::FONT_HERSHEY_SIMPLEX, 0.5, _font_color, 2);
            }
        }
    }
    for (auto& pos_ : _model_pos_B){
        auto py = (int)(pos_.second.x * 10 + width/2 + 300);
        auto px = (int)(pos_.second.y * 10 + height/2);
        if ((px >= 0 && px < width) && (py >= 300 && py < height_new)){
            std::string name_str = pos_.first;
            cv::MarkerTypes type;
            if (name_str.find("car") != std::string::npos){
                type = cv::MARKER_TRIANGLE_UP;
                cv::drawMarker(image, cv::Point(px, py), _B_color, type, 20, 3);
            // name_str = name_str.erase(0, 3);
            cv::putText(image, name_str, cv::Point(px, py+20), cv::FONT_HERSHEY_SIMPLEX, 0.5, _font_color, 2);
        
            }else if(name_str.find("uav") != std::string::npos){
                type = cv::MARKER_STAR;
                cv::drawMarker(image, cv::Point(px, py), _B_color, type, 20, 3);
            // name_str = name_str.erase(0, 3);
            cv::putText(image, name_str, cv::Point(px, py+20), cv::FONT_HERSHEY_SIMPLEX, 0.5, _font_color, 2);
        
            }else{
                type = cv::MARKER_SQUARE;
                cv::rectangle(image,cv::Point(px-20, py-20), cv::Point(px+20, py+20) , _B_color, 3, 8, 0);
                cv::putText(image, "Home", cv::Point(px, py+20), cv::FONT_HERSHEY_SIMPLEX, 0.5, _font_color, 2);
            }
        }
    }

    std::cout.setf(std::ios::fixed);
    std::stringstream ss1, ss2, ss3;
    ss1 << "Body All Blood:";
    ss2 << "Home Blood:";
    double blood_a_body = 0.0;
    double blood_b_body = 0.0;
    double blood_a_home = 0.0;
    double blood_b_home = 0.0;
    auto life_a = _alive_models_A.body;         
    for (size_t i = 0; i < life_a.size(); i++){
        auto bd = life_a[i];
        if (bd.name.data.find("GCS") == std::string::npos){
            if (bd.blood.data > 0){
                blood_a_body += bd.blood.data;
                cv::putText(image, bd.name.data, cv::Point(600, i*50), cv::FONT_HERSHEY_SIMPLEX, 0.5, _A_color, 2);
                cv::line(image, cv::Point(650, i*50), cv::Point(650 + bd.blood.data, i*50), _A_color, 10);
            }
        }else{
            blood_a_home = bd.blood.data;
           
        }
    }

    auto life_b = _alive_models_B.body;
    for (size_t i = 0; i < life_b.size(); i++){
        auto bd = life_b[i];
        // std::cout << "key: " << bd.name.data << " blood: " << bd.blood.data << std::endl;
        if (bd.name.data.find("GCS") == std::string::npos){
            if (bd.blood.data > 0){
                blood_b_body += bd.blood.data;
                cv::putText(image, bd.name.data, cv::Point(250, i*50), cv::FONT_HERSHEY_SIMPLEX, 0.5, _B_color, 2);
                cv::line(image, cv::Point(300, i*50), cv::Point(300 + bd.blood.data, i*50), _B_color, 10);
            }
        }else{
            blood_b_home = bd.blood.data;
        }
    }
    _blood_a_body_remain = blood_a_body;
    _blood_b_body_remain = blood_b_body;
    _home_a_blood_remain = blood_a_home;
    _home_b_blood_remain = blood_b_home;


    double full_blood_body = 1000; //250 pixel
    double full_blood_home = 100;  // 250 pixel
    cv::Point line1_c_up(500,1020 + 300);
    cv::Point line1_c_down(500,1080 + 300);
    cv::Point line2_c_up(500,1120 + 300);
    cv::Point line2_c_down(500,1180 + 300);
    cv::Point line1_l_up;
    cv::Point line1_l_down;
    cv::Point line1_r_up;
    cv::Point line1_r_down;
    cv::Point line2_l_up;
    cv::Point line2_l_down;
    cv::Point line2_r_up;
    cv::Point line2_r_down;

    if(blood_a_body > 1000){
        line1_r_up = cv::Point(750, 1020 + 300);
        line1_r_down = cv::Point(750, 1080 + 300);
    }else{
        int px = 500 + int(blood_a_body/1000.0 * 250);
        line1_r_up = cv::Point(px, 1020 + 300);
        line1_r_down = cv::Point(px, 1080 + 300);
    }
    if(blood_b_body > 1000){
        line1_l_up = cv::Point(250, 1020 + 300);
        line1_l_down = cv::Point(250, 1080 + 300);
    }else{
        int px = 500 - int(blood_b_body/1000.0 * 250);
        line1_l_up = cv::Point(px, 1020 + 300);
        line1_l_down = cv::Point(px, 1080 + 300);
    }

    if(blood_a_home > 100){
        line2_r_up = cv::Point(750, 1120 + 300);
        line2_r_down = cv::Point(750, 1180 + 300);
    }else{
        int px = 500 + int(blood_a_home/100.0 * 250);
        line2_r_up = cv::Point(px, 1120 + 300);
        line2_r_down = cv::Point(px, 1180 + 300);
    }
    if(blood_b_home > 100){
        line2_l_up = cv::Point(250, 1120 + 300);
        line2_l_down = cv::Point(250, 1180 + 300);
    }else{
        int px = 500 - int(blood_b_home/100.0 * 250);
        line2_l_up = cv::Point(px, 1120 + 300);
        line2_l_down = cv::Point(px, 1180 + 300);
    }

    cv::rectangle(image, line1_l_up, line1_c_down ,_B_color,3,8,0);
    cv::Point rookPoints[1][4];
    rookPoints[0][0]  = line1_l_up;
    rookPoints[0][1]  = line1_l_down;
    rookPoints[0][2]  = line1_c_down;;
    rookPoints[0][3]  = line1_c_up;
    const cv::Point* ppt[1]={rookPoints[0]};
	int npt[]={4};
	cv::fillPoly(image, ppt, npt, 1, _B_color, 8);
    cv::putText(image, std::to_string((int)blood_b_body), line1_l_up, cv::FONT_HERSHEY_SIMPLEX, 0.5, _font_color, 2);

    cv::rectangle(image,line1_c_up, line1_r_down ,_A_color,3,8,0);
    rookPoints[0][0]  = line1_c_up;
    rookPoints[0][1]  = line1_c_down;
    rookPoints[0][2]  = line1_r_down;;
    rookPoints[0][3]  = line1_r_up;
	cv::fillPoly(image, ppt, npt, 1, _A_color, 8);
    cv::putText(image, std::to_string((int)blood_a_body), line1_r_up, cv::FONT_HERSHEY_SIMPLEX, 0.5, _font_color, 2);

    cv::rectangle(image,line2_l_up, line2_c_down ,_B_color,3,8,0);
    rookPoints[0][0]  = line2_l_up;
    rookPoints[0][1]  = line2_l_down;
    rookPoints[0][2]  = line2_c_down;;
    rookPoints[0][3]  = line2_c_up;
	cv::fillPoly(image, ppt, npt, 1, _B_color, 8);
    cv::putText(image, std::to_string((int)blood_b_home), line2_l_up, cv::FONT_HERSHEY_SIMPLEX, 0.5, _font_color, 2);

    cv::rectangle(image,line2_c_up, line2_r_down ,_A_color,3,8,0);
    rookPoints[0][0]  = line2_c_up;
    rookPoints[0][1]  = line2_c_down;
    rookPoints[0][2]  = line2_r_down;;
    rookPoints[0][3]  = line2_r_up;
	cv::fillPoly(image, ppt, npt, 1, _A_color, 8);
    cv::putText(image, std::to_string((int)blood_a_home), line2_r_up, cv::FONT_HERSHEY_SIMPLEX, 0.5, _font_color, 2);

    auto t_now = ros::Time::now();
    double timestamp = t_now.toNSec();
    std::string image_name = _res_path + "/" + std::to_string(timestamp) + ".jpg";
    std::string result_name = _res_path + "/" + std::to_string(timestamp) + ".txt";
    std::string work_dirctory;
    work_dirctory = "./";
    std::string file_path_name = work_dirctory + std::to_string(timestamp);
    ss3.str("");
    double t_remain = _time_game_last - (t_now - _start_time).toSec();
    // std::cout << t_now << " : " << t_remain << std::endl;
    int t_min = (int)(t_remain / 60);
    int t_sec = (int)(t_remain - t_min * 60);
    if (_is_start){
        if ((blood_a_home == 0) && (blood_b_home > 0)){
            ss3.str("");
            ss3 << " B team Win !";
            _is_game_over = true;
            _B_win = true;
        }else if ((blood_a_home > 0) && (blood_b_home == 0)){
            ss3.str("");
            ss3 << " A team Win !";
            _is_game_over = true;
            _A_win = true;
        }else if (t_remain < 1.0){
            if ((blood_a_home < blood_b_home)){
                ss3.str("");
                ss3 << " B team Win !";
                _B_win = true; 
            }else if((blood_a_home > blood_b_home)){
                ss3.str("");
                ss3 << " A team Win !";
                _A_win = true;
            }else{
                if (_alive_models_A.body.size() < _alive_models_B.body.size()){
                    ss3.str("");
                    ss3 << " B team Win !";
                    _B_win = true; 
                }else if(_alive_models_A.body.size() > _alive_models_B.body.size()){
                    ss3.str("");
                    ss3 << " A team Win !";
                    _A_win = true;
                }else{
                   if ((blood_a_body < blood_b_body)){
                        ss3.str("");
                        ss3 << " B team Win !";
                        _B_win = true; 
                    }else if((blood_a_body > blood_b_body)){
                        ss3.str("");
                        ss3 << " A team Win !";
                        _A_win = true;
                    }else{
                        ss3.str("");
                        ss3 << "No Winner !";
                    }
                }
                
            }
            _is_game_over = true;
        }else{
            ss3.str("");
            if (t_sec < 10){
                ss3 << t_min << ": 0" << t_sec;
            }else{
                ss3 << t_min << ": " << t_sec;
            }
        }
    }else{
        ss3.str("");
        ss3 << "Starting...";
    }

    cv::putText(image, ss1.str(), cv::Point(60, 1060 + 300), cv::FONT_HERSHEY_SIMPLEX, 0.8, _font_color, 2);
    cv::putText(image, ss2.str(), cv::Point(60, 1160 + 300), cv::FONT_HERSHEY_SIMPLEX, 0.8, _font_color, 2);
    cv::putText(image, ss3.str(), cv::Point(460, 1260 + 300), cv::FONT_HERSHEY_SIMPLEX, 0.8, _font_color, 2);
    std_msgs::Bool game_is_over;
    game_is_over.data = _is_game_over;
    _pub_game_over.publish(game_is_over);
    cv::namedWindow("Mini Map", cv::WINDOW_NORMAL);
    cv::imshow("Mini Map", image);

    if(_is_game_over){
        record_to_file(result_name, t_remain);
        cv::imwrite(image_name, image);
        cv::waitKey(0);
    }else{
        cv::waitKey(10);
    }
    
}

void JudgeGui::record_to_file(std::string file_path_name, double t_remain){
    std::ofstream outfile;
    outfile.open(file_path_name.c_str());
    double a_win = 0.0;
    double b_win = 0.0;
    int count_alive_a;
    int count_alive_b;
    if (_A_win){
        a_win = 1.0;
        b_win = -1.0;
    }else if (_B_win){
        a_win = -1.0;
        b_win = 1.0;
    }else{
        a_win = 0.0;
        b_win = 0.0;
    }
    count_alive_a = _alive_models_A.body.size();
    count_alive_b = _alive_models_B.body.size();
    double a_score = 50.0 * a_win + 3.333 * (count_alive_a - count_alive_b) + 0.02 * (_home_a_blood_remain -_home_b_blood_remain) + t_remain * a_win / 18.0;
    double b_score = 50.0 * b_win + 3.333 * (count_alive_b - count_alive_a) + 0.02 * (_home_b_blood_remain -_home_a_blood_remain) + t_remain * b_win / 18.0;
    ROS_WARN_STREAM("A SCORE: " << a_score);
    ROS_WARN_STREAM("B SCORE: " << b_score);
   
	std::stringstream a_info;
	std::stringstream b_info;
	a_info << "[A Result] WIN? : " << _A_win << std::endl
        << "   alive body: " << count_alive_a << std::endl
        << "   home_blood: " << _home_a_blood_remain << std::endl
        << "   body_blood_all: " << _blood_a_body_remain << std::endl
        << "   time remain: " << (int)t_remain << std::endl
        << "   SCORE : " << a_score << std::endl;
    b_info << "[B Result] WIN? : " << _B_win << std::endl
        << "   alive body: " << count_alive_b << std::endl
        << "   home_blood: " << _home_b_blood_remain << std::endl
        << "   body_blood_all: " << _blood_b_body_remain << std::endl
        << "   time remain: " << (int)t_remain << std::endl
        << "   SCORE : " << b_score << std::endl;
	ROS_WARN_STREAM(a_info.str());
	ROS_WARN_STREAM(b_info.str());
    outfile << a_info.str() << b_info.str() ;
    // outfile << "[A Result] WIN? : " << _A_win 
    //     << " alive body: " << count_alive_a 
    //     << " home_blood: " << _home_a_blood_remain 
    //     << " body_blood_all: " << _blood_a_body_remain
    //     << " time remain: " << t_remain
    //     << " SCORE : " << a_score << std::endl;
    // outfile << "[B Result] WIN? : " << _B_win 
    //     << " alive body: " << count_alive_b 
    //     << " home_blood: " << _home_b_blood_remain 
    //     << " body_blood_all: " << _blood_b_body_remain
    //     << " time remain: " << t_remain
    //     << " SCORE : " << b_score << std::endl;
    outfile.close();
}
} // namespace JudgeSystem
