#pragma once
#include "judge_interface.h"
namespace JudgeSystem
{
class JudgeGui{
public:
    // JudgeGui();
    JudgeGui(ros::NodeHandle& nh, std::string &path);
    ~JudgeGui();
    void set_ros_communication();

    void models_state_callback_a(const gazebo_msgs::ModelStatesConstPtr& msg);
    void models_state_callback_b(const gazebo_msgs::ModelStatesConstPtr& msg);
    void alive_model_callback_a(const judge_system_msgs::Alive_models& msg);
    void alive_model_callback_b(const judge_system_msgs::Alive_models& msg);
    void game_state_callback(const std_msgs::BoolConstPtr& msg);
    void plot_result();
    void record_to_file(std::string file_path_name, double t_remain);
    inline bool is_game_over(){return _is_game_over;};

public:
    double _time_game_last;
    bool _is_start;
    bool _is_game_over;
    bool _A_win;
    bool _B_win;
    std::string _res_path;
    ros::Time _start_time;
    double _home_a_blood_remain;
    double _home_b_blood_remain;
    double _blood_a_body_remain;
    double _blood_b_body_remain;

    judge_system_msgs::Alive_models _alive_models_A;
    judge_system_msgs::Alive_models _alive_models_B;

    ros::NodeHandle _nh;
    ros::Subscriber _sub_models_pose_A;
    ros::Subscriber _sub_models_pose_B;
    ros::Subscriber _sub_alive_model_A;
    ros::Subscriber _sub_alive_model_B;
    ros::Subscriber _sub_game_start;
    ros::Publisher _pub_game_over;

    std::map<std::string, geometry_msgs::Point> _model_pos_A;//
    std::map<std::string, geometry_msgs::Point> _model_pos_B;//

    cv::Scalar _A_color;
    cv::Scalar _B_color;
    cv::Scalar _font_color;
    int plot_cnt;
};
} // namespace JudgeSystem