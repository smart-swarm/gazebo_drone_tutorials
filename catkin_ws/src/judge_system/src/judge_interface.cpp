#include "judge_interface.h"
#include "boost/ref.hpp"
#include "boost/bind.hpp"
#include <iostream>
#include <cstdlib>
#include "geometry_msgs/Vector3.h"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "authorized_check.h"
namespace JudgeSystem
{

JudgeInterface* JudgeInterface::instance()
{
    static JudgeInterface instance;
	return &instance;
}
JudgeInterface::JudgeInterface()
// JudgeInterface::JudgeInterface()
{
    // _A_color = cv::Scalar(0, 0, 255);
    // _B_color = cv::Scalar(255, 0, 0);
    // _font_color = cv::Scalar(0,0,0);
    // plot_cnt = 1;
    // init_judgement();
}

JudgeInterface::~JudgeInterface(){}
void JudgeInterface::init_judgement(ros::NodeHandle& nh){
    // // remote check 
    // _res = 0;
    // _ac.init(std::string("47.93.54.189"), 8080);
    // _ac.gen_request("ecpkn-uav-lab");
    // _res = _ac.run_check();
    // std::cout << " remote server check: " << _res << std::endl;
    // if (_res == 0){
    //     std::cout << "Can not connect to server. please check your network or read the instruction of this software.";
    //     return ;
    // }
    _nh = nh;
    ros::start();
    ros::Time::init();
    ROS_INFO_STREAM("Start initializing judge system interface");
    
    _latest_time = ros::Time::now();
    _is_start = false;
    _is_game_over = false;
    ros::NodeHandle pnh("~");
    pnh.param("bullet_limit", _bullet_capacity_limit, 20); 
    pnh.param("bullet_model_count", _bullet_model_count, 5); 
    pnh.param("car_count_a", _car_count_a, 2); 
    pnh.param("car_count_b", _car_count_b, 2); 
    pnh.param("uav_count_a", _uav_count_a, 3); 
    pnh.param("uav_count_b", _uav_count_b, 3); 
    pnh.param("blood_full", _blood_full, 100.0); 
    pnh.param("speed_bullet", _speed_bullet, 10.0); 
    pnh.param("decay_factor", _decay_factor, 1.0); 
    pnh.param("boom_raidus", _boom_raidus, 1.0); 
    pnh.param("attack_duration", _attack_duration, 0.2); 
    pnh.param("bullet_power_factor", _bullet_power_factor, 0.5); 
    pnh.param("bullet_power_base", _bullet_power_base, 20.0); 
    pnh.param("time_game_last", _time_game_last, 150.0); 
    _bullet_power_factor_dynamic = _bullet_power_factor;
    ROS_INFO_STREAM( "_bullet_capacity_limit : " << _bullet_capacity_limit); 

    _models_B.clear();
    _models_A.clear();
    
    _models_attacked_event_B.clear();
    _models_attacked_event_A.clear();
    _bullet_alive.clear();
    _model_pos_A.clear();
    _model_pos_B.clear();
    _model_vel_A.clear();
    _model_vel_B.clear();
    _bullet_model_name.clear();

    std::set<std::string> uav_name_A; //
    std::set<std::string> uav_name_B; //
    std::set<std::string> car_name_A; //
    std::set<std::string> car_name_B; //
    for (int i = 0; i < _car_count_a; i++){
        std::stringstream name_A;
        name_A << "carA" << (i + 1);
        car_name_A.insert(name_A.str());
        std::stringstream cmd_vel_name_A;
        cmd_vel_name_A << "/A/car" << (i + 1) << "/cmd_vel";
        ros::Publisher pub_car_A = _nh.advertise<geometry_msgs::Twist>(cmd_vel_name_A.str(), 1);
        _models_move_cmd_A[name_A.str()] = pub_car_A;
    }
    for (int i = 0; i < _car_count_b; i++){
        std::stringstream name_B;
        name_B << "carB" << (i + 1);
        car_name_B.insert(name_B.str());
        std::stringstream cmd_vel_name_B;
        cmd_vel_name_B << "/B/car" << (i + 1) << "/cmd_vel";
        ros::Publisher pub_car_B = _nh.advertise<geometry_msgs::Twist>(cmd_vel_name_B.str(), 1);
        _models_move_cmd_B[name_B.str()] = pub_car_B;
    }

    for (int i = 0; i < _uav_count_a; i++){
        std::stringstream name_A;
        name_A << "uavA" << (i + 1);
        uav_name_A.insert(name_A.str());
        std::stringstream cmd_vel_name_A;
        cmd_vel_name_A << "/A/uav" << (i + 1) << "/velocity_cmd";
        ros::Publisher pub_uav_A = _nh.advertise<geometry_msgs::Twist>(cmd_vel_name_A.str(), 1);
        _models_move_cmd_A[name_A.str()] = pub_uav_A;
    }
    for (int i = 0; i < _uav_count_b; i++){
        std::stringstream name_B;
        name_B << "uavB" << (i + 1);
        uav_name_B.insert(name_B.str());
        std::stringstream cmd_vel_name_B;
        cmd_vel_name_B << "/B/uav" << (i + 1) << "/velocity_cmd";
        ros::Publisher pub_uav_B = _nh.advertise<geometry_msgs::Twist>(cmd_vel_name_B.str(), 1);
        _models_move_cmd_B[name_B.str()] = pub_uav_B;
    }

    // merge car and uav name.
    // _models_bullet.clear();
    // _models_body.clear();
    // _models_bullet_launch_pub.clear();
    _models_A = car_name_A;
    _models_A.insert(uav_name_A.begin(), uav_name_A.end());
    // _models_body = _models_A;
    
    _models_A.insert("GCSA0");
    _models_B = car_name_B;
    _models_B.insert(uav_name_B.begin(), uav_name_B.end());
    
    // _models_body.insert(_models_B.begin(), _models_B.end());
    _models_B.insert("GCSB0");

    // for (auto& body_name : _models_body){
    //     for (int i = 0; i < _bullet_model_count; i++){
    //         _models_bullet.insert(body_name + "_bullet_" + std::to_string(i + 1));
    //     }
    // }
    // for (auto& bullet_name : _models_bullet){
    //     ros::Publisher pub_bullet_launch = _nh.advertise<geometry_msgs::Vector3>(bullet_name + "/vel/cmd", 1);
    //     _models_bullet_launch_pub[bullet_name] = pub_bullet_launch;
    // }   
    // load bullet and full blood.
    for (auto& nm : _models_A){
        ModelAttackedEvent event_record;
        event_record.name = nm;
        if (nm.find("GCS") != std::string::npos){
            event_record.blood_remain = 10.0 * _blood_full;
        }else{
            event_record.blood_remain = _blood_full;
        }
        
        event_record.bullet_remain = _bullet_capacity_limit;
        event_record.decay_factor = _decay_factor;
        event_record.latest_attacked_time = ros::Time(0);
        _models_attacked_event_A[nm] = event_record;
    }
    for (auto& nm : _models_B){
        ModelAttackedEvent event_record;
        event_record.name = nm;
        if (nm.find("GCS") != std::string::npos){
            event_record.blood_remain = 10.0 * _blood_full;
        }else{
            event_record.blood_remain = _blood_full;
        }
        event_record.bullet_remain = _bullet_capacity_limit;
        event_record.decay_factor = _decay_factor;
        event_record.latest_attacked_time = ros::Time(0);
        _models_attacked_event_B[nm] = event_record;
    }
    for (auto& model : _models_attacked_event_A){   
        if (model.second.blood_remain > 0){
            judge_system_msgs::Alive_model alive_m;
            alive_m.blood.data = model.second.blood_remain;
            alive_m.bullet.data = model.second.bullet_remain;
            alive_m.name.data = model.first;
            _alive_models_A.body.push_back(alive_m);
        }
    } 
    for (auto& model : _models_attacked_event_B){   
        if (model.second.blood_remain > 0){
            judge_system_msgs::Alive_model alive_m;
            alive_m.blood.data = model.second.blood_remain;
            alive_m.bullet.data = model.second.bullet_remain;
            alive_m.name.data = model.first;
            _alive_models_B.body.push_back(alive_m);
        }
    }
    JudgeInterface::set_ros_communication();
}

void JudgeInterface::set_ros_communication(){

    // _deleteModelClient = _nh.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
    _model_state_client = _nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state", true);
    _sub_models_state_ALL = _nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 30,
            boost::bind(&JudgeSystem::JudgeInterface::models_state_callback, this, _1));

    _sub_move_cmd_A = _nh.subscribe("/A/move_cmd", 10, &JudgeSystem::JudgeInterface::move_cmd_callback, this);
    _sub_move_cmd_B = _nh.subscribe("/B/move_cmd", 10, &JudgeSystem::JudgeInterface::move_cmd_callback, this);

    _sub_attack_cmd_A = _nh.subscribe("/A/attack_cmd", 1, &JudgeSystem::JudgeInterface::attack_cmd_callback, this);
    _sub_attack_cmd_B = _nh.subscribe("/B/attack_cmd", 1, &JudgeSystem::JudgeInterface::attack_cmd_callback, this);
    _sub_game_over_status = _nh.subscribe("/game_over_status", 10, &JudgeSystem::JudgeInterface::game_over_status_callback, this);
    
    _pub_model_pose_A = _nh.advertise<gazebo_msgs::ModelStates>("/A/car_uav_pose", 1);
    _pub_model_pose_B = _nh.advertise<gazebo_msgs::ModelStates>("/B/car_uav_pose", 1);

    _pub_alive_model_A = _nh.advertise<judge_system_msgs::Alive_models>("/A/alive_model", 1);
    _pub_alive_model_B = _nh.advertise<judge_system_msgs::Alive_models>("/B/alive_model", 1);
    _pub_game_start = _nh.advertise<std_msgs::Bool>("/game_start_status", 1);
    _clear_all_bullet = _nh.advertiseService("/reset_plat", &JudgeSystem::JudgeInterface::clear_bullet, this);
    ros::service::waitForService("gazebo/set_model_state", -1);
}

void JudgeInterface::game_over_status_callback(const std_msgs::BoolConstPtr& msg){
    _is_game_over = msg->data;
}

void JudgeInterface::models_state_callback(const gazebo_msgs::ModelStatesConstPtr& msg){
    _all_models_state = *msg;
    // JudgeInterface::publish_pose_model_state();
}

void JudgeInterface::publish_pose_model_state(){
    _latest_time = ros::Time::now();
    auto name_list = _all_models_state.name;
    auto pose_list = _all_models_state.pose;
    auto twist_list = _all_models_state.twist;
    int count =  name_list.size();
    gazebo_msgs::ModelStates A_model_pose; //
    gazebo_msgs::ModelStates B_model_pose; //
    for (int i = 0; i < count; i++){
        std::string name_tmp = name_list[i];
        if (JudgeInterface::element_in_set(name_tmp, _models_A)){
            _model_pos_A[name_tmp] = pose_list[i].position;
            _model_vel_A[name_tmp] = twist_list[i].linear;
            A_model_pose.name.push_back(name_tmp);
            A_model_pose.pose.push_back(pose_list[i]);
            A_model_pose.twist.push_back(twist_list[i]);
        }
        else if (JudgeInterface::element_in_set(name_tmp, _models_B)){
            _model_pos_B[name_tmp] = pose_list[i].position;
            _model_vel_B[name_tmp] = twist_list[i].linear;
            B_model_pose.name.push_back(name_tmp);
            B_model_pose.pose.push_back(pose_list[i]);
            B_model_pose.twist.push_back(twist_list[i]);
        }
        else if(name_tmp.find("bullet") != std::string::npos){
            if (fabs(pose_list[i].position.x) < 8.0 && fabs(pose_list[i].position.y) < 9.0 && fabs(pose_list[i].position.z) < 7.5){
                delete_model(name_tmp);
                continue;
            }
            if (fabs(pose_list[i].position.y) < 19.5 && fabs(pose_list[i].position.x) < 29.5){
                if (fabs(pose_list[i].position.z) > 0.1 && fabs(pose_list[i].position.z) < 15.0){
                    _bullet_alive[name_tmp] = pose_list[i].position;
                }else{
                    delete_model(name_tmp);
                }
                continue;
            }
            if(fabs(pose_list[i].position.y) < 200.0 && fabs(pose_list[i].position.x) < 200.0){
                delete_model(name_tmp);
                continue;
            }
        }
        else{
            // LOG(INFO) << "model is built in gazebo." << std::endl;
            continue;
        }
    }
    _pub_model_pose_A.publish(A_model_pose);
    _pub_model_pose_B.publish(B_model_pose);
    _A_model_pose_pub = A_model_pose;
    _B_model_pose_pub = B_model_pose;
    JudgeInterface::publish_alive_model();
    JudgeInterface::update_life();  

}

void JudgeInterface::move_cmd_callback(const judge_system_msgs::Move_cmd& msg){
    std::string team = msg.team.data;
    auto name = msg.name;
    auto cmd_all = msg.cmd;
    int count = name.size();
    if (name.size() != cmd_all.size()){
        ROS_ERROR_STREAM("move cmd format error!");
        return ;
    }
    if (cmd_all.empty()){
         ROS_ERROR_STREAM("move cmd is empty.");
        return;
    }
    for (int i = 0; i < count; i++){
        if ((name[i].data.find("car") == std::string::npos) && (name[i].data.find("uav") == std::string::npos)){
            continue;
        }
        if (team == "A"){
            if (JudgeInterface::check_model_alive(team, name[i].data) && (!_is_game_over)){
                _models_move_cmd_A.at(name[i].data).publish(cmd_all[i]);
            }else{
                geometry_msgs::Twist cmd_zero;
                _models_move_cmd_A.at(name[i].data).publish(cmd_zero);
            }
        }
        if (team == "B"){
            if (JudgeInterface::check_model_alive(team, name[i].data) && (!_is_game_over)){
                _models_move_cmd_B.at(name[i].data).publish(cmd_all[i]);
            }else{
                geometry_msgs::Twist cmd_zero;
                _models_move_cmd_B.at(name[i].data).publish(cmd_zero);
            }
        }
    }
    if (!_is_start){
        _start_time = ros::Time::now();
        _is_start = true;
    }
    if (_is_start){
        std_msgs::Bool game_start;
        game_start.data = true;
        _pub_game_start.publish(game_start);
    }
}

void JudgeInterface::attack_cmd_callback(const judge_system_msgs::Attack_cmd& msg){
    std::string team = msg.team.data;
    std::string model_name = msg.name.data;
    auto start_pt = msg.start;
    auto target_pt = msg.target;
    if ((model_name.find("car") == std::string::npos) && (model_name.find("uav") == std::string::npos)){
        return;
    }

    if (team == "A"){
        if (check_bullet_remain(team, model_name) && (!_is_game_over)){
            auto latest_attacked_time = _models_attacked_event_A[model_name].latest_attacked_time;
            // if ((ros::Time::now() - latest_attacked_time).toSec < _attack_duration){
            //     ROS_INFO_STREAM(model_name << ": waiting for process of cool finish.");
            //     return ;
            // }
            int remain_bullet = _models_attacked_event_A[model_name].bullet_remain;
            auto bullet_name = model_name + "_bullet_" + std::to_string(remain_bullet % _bullet_model_count + 1);
            // auto vel_model = _model_vel_A.at(model_name);
            // double delta_t = 0.5;
            // start_pt.x = start_pt.x + vel_model.x * delta_t;
            // start_pt.y = start_pt.y + vel_model.y * delta_t;
            // start_pt.z = start_pt.z + vel_model.z * delta_t;
            JudgeInterface::launch_bullet(bullet_name, start_pt, target_pt); 
            // ROS_INFO_STREAM("A: " << bullet_name);
            _models_attacked_event_A[model_name].bullet_remain = remain_bullet - 1;
            _bullet_alive[bullet_name] = start_pt;
        }else{
            ROS_INFO_STREAM(model_name << ": bullets run out. ");
        }
    }
    if (team == "B"){
        if (check_bullet_remain(team, model_name) && (!_is_game_over)){
            auto latest_attacked_time = _models_attacked_event_B[model_name].latest_attacked_time;
            // if ((ros::Time::now() - latest_attacked_time).toSec < _attack_duration){
            //     ROS_INFO_STREAM(model_name << ": waiting for process of cool finish.");
            //     return ;
            // }
            int remain_bullet = _models_attacked_event_B[model_name].bullet_remain;
            auto bullet_name = model_name + "_bullet_" + std::to_string(remain_bullet % _bullet_model_count + 1);
            // auto vel_model = _model_vel_B.at(model_name);
            // double delta_t = 0.4;
            // start_pt.x = start_pt.x + vel_model.x * delta_t;
            // start_pt.y = start_pt.y + vel_model.y * delta_t;
            // start_pt.z = start_pt.z + vel_model.z * delta_t;
            JudgeInterface::launch_bullet(bullet_name, start_pt, target_pt); 
            _models_attacked_event_B[model_name].bullet_remain = remain_bullet - 1;
            _bullet_alive[bullet_name] = start_pt;
        }else{
            ROS_INFO_STREAM(model_name << ": bullets run out. ");
        }
    }
    // JudgeInterface::update_life();
}

void JudgeInterface::update_life(){
    // std::cout << " Update all model life. bullet number is :" << _bullet_alive.size() << std::endl;
    
    if (_bullet_alive.size() < 1){
        // std::cout << " There is no valid bullet flying." << std::endl; 
        return;
    }
    _alive_models_A.team.data = "A";
    _alive_models_A.body.clear();
    _alive_models_B.team.data = "B";
    _alive_models_B.body.clear();
    auto bullet = _bullet_alive.begin();
    while(bullet !=_bullet_alive.end()){
    // for (auto bullet = _bullet_alive.begin(); bullet !=_bullet_alive.end();){
        if (bullet->second.z < 0.1){
            JudgeInterface::delete_model(bullet->first);
            bullet = _bullet_alive.erase(bullet);
            continue;
        }
    
        bool flag_boom = false;

        for (auto& model : _model_pos_A){
            if (bullet->first.find(model.first) != std::string::npos){
                continue;
            }
            auto dist = JudgeInterface::dist_bullet2model(bullet->second, model.second);
            bool judge = dist < _boom_raidus;
            
            if (model.first.find("GCS") != std::string::npos){
                // ROS_INFO_STREAM(model.first << " dist: " << dist << std::endl);
                if (dist < 6.5){
                    judge = true;
                }
            }
            if (judge){
                auto blood = _models_attacked_event_A[model.first].blood_remain;
                auto new_attack_time = ros::Time::now();
                auto latest_attacked_time = _models_attacked_event_A[model.first].latest_attacked_time;
                auto decay_f = _models_attacked_event_A[model.first].decay_factor;
                double dt = new_attack_time.toSec() - latest_attacked_time.toSec();
                if (dt < _attack_duration && model.first.find("GCS") == std::string::npos){
                    decay_f += _bullet_power_factor;
                }else{
                    decay_f += _bullet_power_factor;
                }
                if (decay_f > 3.0){
                    decay_f = 3.0;
                }
                blood = blood - _bullet_power_base * decay_f;
                _models_attacked_event_A[model.first].blood_remain = blood;
                _models_attacked_event_A[model.first].decay_factor = decay_f;
                _models_attacked_event_A[model.first].latest_attacked_time = new_attack_time;
                auto bullet_name = bullet->first;
                ROS_WARN_STREAM(model.first << " is attacked by " << bullet_name.substr(0, 5) << std::endl);
                if (fabs(blood) < 0.001){
                    std::stringstream arm_cmd;
                    auto name = model.first;
                    if (name.find("GCS") == std::string::npos){
                        std::string team;
                        team = "A";
                        stop_body(team, name);
                        if (name.find("uav") != std::string::npos){
                            drop_down_uav(name, team);
                        }
                        ROS_WARN_STREAM(name << " is destroyed!" << std::endl);
                    }
                }
                flag_boom = true;
                break;
            }
        }
        for (auto& model : _model_pos_B){
            if (bullet->first.find(model.first) != std::string::npos){
                continue;
            }
            auto dist = JudgeInterface::dist_bullet2model(bullet->second, model.second);
            bool judge = dist < _boom_raidus;
            
            if (model.first.find("GCS") != std::string::npos){
                // ROS_INFO_STREAM(model.first << " dist: " << dist << std::endl);
                if (dist < 6.5){
                    judge = true;
                }
            }
            if (judge){
                auto blood = _models_attacked_event_B[model.first].blood_remain;
                auto new_attack_time = ros::Time::now();
                auto latest_attacked_time = _models_attacked_event_B[model.first].latest_attacked_time;
                auto decay_f = _models_attacked_event_B[model.first].decay_factor;
                double dt = new_attack_time.toSec() - latest_attacked_time.toSec();
                if (dt < _attack_duration && model.first.find("GCS") == std::string::npos){
                    decay_f += _bullet_power_factor;
                }else{
                    decay_f += _bullet_power_factor;
                }
                if (decay_f > 3.0){
                    decay_f = 3.0;
                }
                blood = blood - _bullet_power_base * decay_f;
                _models_attacked_event_B[model.first].blood_remain = blood;
                _models_attacked_event_B[model.first].decay_factor = decay_f;
                _models_attacked_event_B[model.first].latest_attacked_time = new_attack_time;
                auto bullet_name = bullet->first;
                ROS_WARN_STREAM(model.first << " is attacked by " << bullet_name.substr(0, 5) << std::endl);
                if (fabs(blood) < 0.001){
                    std::stringstream arm_cmd;
                    auto name = model.first;
                    if (name.find("GCS") == std::string::npos){
                        std::string team;
                        team = "B";
                        stop_body(team, name);
                        if (name.find("uav") != std::string::npos){
                            drop_down_uav(name, team);
                        }
                        ROS_WARN_STREAM(name << " is destroyed!" << std::endl);
                    }
                }
                flag_boom = true;
                break;
            }
        }
        if (flag_boom){
            JudgeInterface::delete_model(bullet->first);
            bullet = _bullet_alive.erase(bullet);
            continue;
        }
        bullet++;
    }

    for (auto& model : _models_attacked_event_A){   
        if (model.second.blood_remain > 0){
            judge_system_msgs::Alive_model alive_m;
            alive_m.blood.data = model.second.blood_remain;
            alive_m.bullet.data = model.second.bullet_remain;
            alive_m.name.data = model.first;
            _alive_models_A.body.push_back(alive_m);
        }
    } 
    for (auto& model : _models_attacked_event_B){   
        if (model.second.blood_remain > 0){
            judge_system_msgs::Alive_model alive_m;
            alive_m.blood.data = model.second.blood_remain;
            alive_m.bullet.data = model.second.bullet_remain;
            alive_m.name.data = model.first;
            _alive_models_B.body.push_back(alive_m);
        }
    }
}

void JudgeInterface::publish_alive_model(){
    _pub_alive_model_A.publish(_alive_models_A);
    _pub_alive_model_B.publish(_alive_models_B);
}

bool JudgeInterface::element_in_set(const std::string& elem, const std::set<std::string>& set_data){
    if (set_data.find(elem) != set_data.end()){
        return true;
    }
    return false;
}

bool JudgeInterface::launch_bullet(const std::string& model_name, 
                                    const geometry_msgs::Point& start, 
                                    const geometry_msgs::Point& target){
    std::string reference_frame = "link";
    geometry_msgs::Pose start_pose;
    geometry_msgs::Twist start_twist;
    if (model_name.find("uav") != std::string::npos){
        start_pose.position = start;
        start_pose.position.z = start.z - 0.5;
    }else{
        start_pose.position = start;
        start_pose.position.x = start.x + 0.5;
    }

    geometry_msgs::Vector3 d_norm;
    geometry_msgs::Vector3 vel;
    d_norm.x = target.x - start_pose.position.x;
    d_norm.y = target.y - start_pose.position.y;
    d_norm.z = target.z - start_pose.position.z;
    
    auto dist = sqrt(d_norm.x * d_norm.x + d_norm.y * d_norm.y + d_norm.z * d_norm.z);
    d_norm.x =  d_norm.x / dist;
    d_norm.y =  d_norm.y / dist;
    d_norm.z =  d_norm.z / dist;

    vel.x =  d_norm.x * _speed_bullet;
    vel.y =  d_norm.y * _speed_bullet;
    vel.z =  d_norm.z * _speed_bullet;

    std::stringstream fly_cmd;
    fly_cmd << "rostopic pub --once /" << model_name 
        <<"/vel_cmd geometry_msgs/Vector3 " 
        << "\'{x: " << vel.x << ", y: " << vel.y << ", z: " << vel.z << "}\' & "
        << std::endl;
    // ROS_INFO_STREAM(fly_cmd.str());
    system(fly_cmd.str().c_str());
    usleep(20000);
    gazebo_msgs::ModelState target_model_state;
    gazebo_msgs::SetModelState set_model_state;
    target_model_state.model_name = model_name;
    target_model_state.reference_frame = reference_frame;
    target_model_state.pose = start_pose;
    target_model_state.twist = start_twist;
    set_model_state.request.model_state = target_model_state;
    _model_state_client.call(set_model_state);
    system(fly_cmd.str().c_str());
    _bullet_model_name.push_back(model_name);
   
}

bool JudgeInterface::delete_model(const std::string& model_name)
{
    std::string reference_frame = "link";
    geometry_msgs::Pose start_pose;
    // start_pose.position.x = 1000.0 + rand()%50;
    // start_pose.position.y = 1000.0 + rand()%50;
    start_pose.position.x = 1000.0;
    start_pose.position.y = 1000.0;
    start_pose.position.y = 10.0;
    geometry_msgs::Twist start_twist;

    gazebo_msgs::ModelState target_model_state;
    gazebo_msgs::SetModelState set_model_state;

    target_model_state.model_name = model_name;
    target_model_state.reference_frame = reference_frame;
    target_model_state.pose = start_pose;
    target_model_state.twist = start_twist;

    set_model_state.request.model_state = target_model_state;
    _model_state_client.call(set_model_state);
}

void JudgeInterface::drop_down_uav(std::string &uav_name, std::string &team){
    std::stringstream arm_cmd;
    stop_body(team, uav_name);
    if (team == "A"){
        uav_name.erase(std::remove(uav_name.begin(), uav_name.end(), 'A'), uav_name.end());
        arm_cmd << "rosservice call /" + team + "/" + uav_name + "/enable_motors false " << std::endl;
    }else if (team == "B"){
        uav_name.erase(std::remove(uav_name.begin(), uav_name.end(), 'B'), uav_name.end());
        arm_cmd << "rosservice call /" + team + "/" + uav_name + "/enable_motors false " << std::endl;
    }else{
        ROS_INFO_STREAM("Team name error!");
        return ;
    }
    // ROS_INFO_STREAM(arm_cmd.str());
    system(arm_cmd.str().c_str());
    // ROS_ERROR_STREAM(uav_name << " is destroyed !" << std::endl);
}

bool JudgeInterface::check_model_alive(const std::string& team, const std::string& name){
    if (team == "A"){
        return (_models_attacked_event_A[name].blood_remain > 0) ;
    }
    if (team == "B"){
        return (_models_attacked_event_B[name].blood_remain > 0) ;
    }
    return false;
}

bool JudgeInterface::check_bullet_remain(const std::string& team, const std::string& name){
    if (team == "A"){
        return (_models_attacked_event_A[name].blood_remain > 0) && (_models_attacked_event_A[name].bullet_remain > 0) ;
    }
    if (team == "B"){
        return (_models_attacked_event_B[name].blood_remain > 0) && (_models_attacked_event_B[name].bullet_remain > 0) ;
    }
    return false;
}

double JudgeInterface::dist_bullet2model(const geometry_msgs::Point& bullet, const geometry_msgs::Point& model){
    double dx = bullet.x - model.x;
    double dy = bullet.y - model.y;
    double dz = bullet.z - model.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

bool JudgeInterface::clear_bullet(judge_system_msgs::clear_bullets::Request  &req, judge_system_msgs::clear_bullets::Response &res){
    if (req.clear){
        for (auto& name : _bullet_model_name){
            JudgeInterface::delete_model(name);
        }
        stop_all_body();
        // JudgeInterface::init_judgement();
        res.ok = true;
        return true;
    }
    res.ok = false;
    return false;
}

bool JudgeInterface::stop_all_body(){
    std::string team;
    for (auto& name : _models_A){
        if (name.find("GCS") != std::string::npos){
            continue;
        }
        team = "A";
        stop_body(team, name);
    }
    for (auto& name : _models_B){
        if (name.find("GCS") != std::string::npos){
            continue;
        }
        team = "B";
        stop_body(team, name);
    }
    return true;
}   

bool JudgeInterface::stop_body(std::string &team, std::string name){
    if (team == "A"){
        geometry_msgs::Twist cmd_zero;
        _models_move_cmd_A.at(name).publish(cmd_zero);
        return true;
    }
    if (team == "B"){
        geometry_msgs::Twist cmd_zero;
        _models_move_cmd_B.at(name).publish(cmd_zero);
        return true;
    }
    return false;
}

std::map<std::string, geometry_msgs::Point> JudgeInterface::get_models_pose(std::string &team){
    if (team=="a" || team=="A"){
        return _model_pos_A;
    }else if(team=="b" || team=="B"){
        return _model_pos_B;
    }else{
        ROS_ERROR_STREAM("tean name error. team can be only a or b.");
    }
}

judge_system_msgs::Alive_models JudgeInterface::get_alive_models(std::string &team){
    if (team=="a"){
        return _alive_models_A;
    }else if(team=="b"){
        return _alive_models_B;
    }else{
        ROS_ERROR_STREAM("tean name error. team can be only a or b." );
    }
}

void JudgeInterface::ensure_dead_uav_down(){
    for (auto& model_event : _models_attacked_event_A){
        if (model_event.second.blood_remain < 1.0f){
            auto name = model_event.first;
            ROS_ERROR_STREAM(name << " is destroyed !" << std::endl);
            std::string team;
            team = "A";
            if (name.find("uav") != std::string::npos){
                drop_down_uav(name, team);
            }
        }
    }
    for (auto& model_event : _models_attacked_event_B){
        if (model_event.second.blood_remain < 1.0f){
            auto name = model_event.first;
            ROS_ERROR_STREAM(name << " is destroyed !" << std::endl);
            std::string team;
            team = "B";
            if (name.find("uav") != std::string::npos){
                drop_down_uav(name, team);
            }
        }
    }
}

} // namespace JudgeSystem