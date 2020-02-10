
#include "judge_gui.h"
int main(int argc, char** argv)
{
    ros::init(argc, argv, "judge_system_gui");
    ros::NodeHandle nh;
    ros::Rate r(30);
    std::string res_path;
    if (argc > 1) {
		res_path = std::string(argv[1]);
	}else
    {
        res_path = ".";
    }
    ROS_WARN_STREAM("result save path:  " << res_path);
    JudgeSystem::JudgeGui plot_gui(nh, res_path);
    while(!plot_gui.is_game_over()){
        plot_gui.plot_result();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
