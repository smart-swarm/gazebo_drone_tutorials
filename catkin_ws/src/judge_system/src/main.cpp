#include "judge_interface.h"
#include <iostream>
#include <ros/node_handle.h>

int main(int argc, char** argv)
{  
    ros::init(argc, argv, "judge_system");
    ros::NodeHandle nh;
    JudgeSystem::JudgeInterface::instance()->init_judgement(nh);
    ros::Rate r(30);
    while(ros::ok()){
        ros::spinOnce();
        JudgeSystem::JudgeInterface::instance()->publish_pose_model_state();
        // JudgeSystem::JudgeInterface::instance()->ensure_dead_uav_down();
        r.sleep();
    }
    // ros::spin();
    return 0;
}
