#!/bin/bash

sh python_kill.sh
source $(pwd)/devel/setup.bash
roslaunch sim_platform_pysdk start_computer_and_your_code.launch
