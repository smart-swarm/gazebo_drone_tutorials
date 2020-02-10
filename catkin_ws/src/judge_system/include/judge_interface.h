#pragma once
#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "gazebo_msgs/ModelStates.h"
#include "judge_system_msgs/Move_cmd.h"
#include "judge_system_msgs/Alive_model.h"
#include "judge_system_msgs/Alive_models.h"
#include "judge_system_msgs/Bullet_launch.h"
#include "judge_system_msgs/clear_bullets.h"
#include "judge_system_msgs/Attack_cmd.h"
#include "gazebo_msgs/DeleteModel.h"
#include "authorized_check.h"
namespace JudgeSystem
{
typedef struct ModelAttackedEvent{
    std::string name;
    double blood_remain;
    int bullet_remain;
    double decay_factor;
    ros::Time latest_attacked_time;
}ModelAttackedEvent;

class JudgeInterface{
public:
    JudgeInterface();
    ~JudgeInterface();
    static JudgeInterface* instance();
    void set_ros_communication();
    void init_judgement(ros::NodeHandle& nh);
    void update_life();

    void models_state_callback(const gazebo_msgs::ModelStatesConstPtr& msg);
    void move_cmd_callback(const judge_system_msgs::Move_cmd& msg);
    void attack_cmd_callback(const judge_system_msgs::Attack_cmd& msg);
    void game_over_status_callback(const std_msgs::BoolConstPtr& msg);

    void publish_pose_model_state();

    void publish_alive_model();
    // void plot_result();
    bool clear_bullet(judge_system_msgs::clear_bullets::Request  &req, judge_system_msgs::clear_bullets::Response &res);
    bool stop_body(std::string &team, std::string name);
    bool stop_all_body();
    std::map<std::string, geometry_msgs::Point> get_models_pose(std::string &team);
    judge_system_msgs::Alive_models get_alive_models(std::string &team);
    inline bool get_game_state(){ return _is_start;};
    void drop_down_uav(std::string &uav_name, std::string &team);
    void ensure_dead_uav_down();
// private:
public:
    bool launch_bullet(const std::string& model_name, 
                                const geometry_msgs::Point& start, 
                                const geometry_msgs::Point& target);
    bool delete_model(const std::string& model_name);
     bool element_in_set(const std::string& elem, const std::set<std::string>& set_data);
    bool check_model_alive(const std::string& team, const std::string& name);
    bool check_bullet_remain(const std::string& team, const std::string& name);
    double dist_bullet2model(const geometry_msgs::Point& bullet, const geometry_msgs::Point& model);
public:
	AuthorizedCheck _ac;
	int32_t _res;
    ros::Time _latest_time;
    int _bullet_capacity_limit;
    int _bullet_model_count;
    int _car_count_a; // 
    int _car_count_b; //
    int _uav_count_a;
    int _uav_count_b;
    double _blood_full; //
    double _decay_factor; //
    double _boom_raidus; //
    double _attack_duration;//
    double _bullet_power_factor;
    double _bullet_power_factor_dynamic;
    double _bullet_power_base;
    double _time_game_last;
    bool _is_start;
    bool _is_game_over;
    ros::Time _start_time;

    gazebo_msgs::ModelStates _all_models_state;
    judge_system_msgs::Move_cmd _move_cmd_a_msg_sub;
    judge_system_msgs::Move_cmd _move_cmd_b_msg_sub;
    judge_system_msgs::Attack_cmd _attack_cmd_a_msg_sub;
    judge_system_msgs::Attack_cmd _attack_cmd_b_msg_sub;
    
    gazebo_msgs::ModelStates _A_model_pose_pub; //
    gazebo_msgs::ModelStates _B_model_pose_pub; //

    judge_system_msgs::Alive_models _alive_models_A;
    judge_system_msgs::Alive_models _alive_models_B;

    ros::NodeHandle _nh;
    ros::Subscriber _sub_models_state_ALL;
    ros::Subscriber _sub_move_cmd_A;
    ros::Subscriber _sub_attack_cmd_A;
    ros::Subscriber _sub_move_cmd_B;
    ros::Subscriber _sub_attack_cmd_B;

    ros::Publisher _pub_model_pose_A;
    ros::Publisher _pub_model_pose_B;
    ros::Publisher _pub_game_start;

    ros::Publisher _pub_alive_model_A;
    ros::Publisher _pub_alive_model_B;

    std::map<std::string, ros::Publisher> _models_move_cmd_A; //
    std::map<std::string, ros::Publisher> _models_move_cmd_B; //
    // std::map<std::string, ros::Publisher> _models_bullet_launch_pub; //

    std::set<std::string> _models_A; //
    std::set<std::string> _models_B; //
    // std::set<std::string> _models_body; //
    // std::set<std::string> _models_bullet; //

    std::map<std::string, geometry_msgs::Point> _bullet_alive;//
    std::map<std::string, geometry_msgs::Point> _model_pos_A;//
    std::map<std::string, geometry_msgs::Point> _model_pos_B;//

    std::map<std::string, geometry_msgs::Vector3> _model_vel_A;//
    std::map<std::string, geometry_msgs::Vector3> _model_vel_B;//

    std::map<std::string, ModelAttackedEvent> _models_attacked_event_A;
    std::map<std::string, ModelAttackedEvent> _models_attacked_event_B; //

    ros::ServiceClient _deleteModelClient; //
    gazebo_msgs::DeleteModel _deleteModel; //

    ros::ServiceClient _model_state_client;
    ros::Subscriber _sub_game_over_status;
    

    std::vector<std::string> _bullet_model_name;
    ros::ServiceServer _clear_all_bullet;
    double _speed_bullet;

};
} // namespace JudgeSystem
