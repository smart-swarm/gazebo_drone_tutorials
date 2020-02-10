#!/bin/bash
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'carA1'
  pose:
    position:
      x: 10.0
      y: 9.0
      z: 0
    orientation:
      x: 0.0
      y: 0.0
      z: 1.0
      w: 0.0
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: 'link'" 

rosservice call /gazebo/set_model_state "model_state:
  model_name: 'carA2'
  pose:
    position:
      x: 10.0
      y: -9.0
      z: 0
    orientation:
      x: 0.0
      y: 0.0
      z: 1.0
      w: 0.0
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: 'link'" 

rosservice call /gazebo/set_model_state "model_state:
  model_name: 'carA3'
  pose:
    position:
      x: 20.0
      y: 0.0
      z: 0
    orientation:
      x: 0.0
      y: 0.0
      z: 1.0
      w: 0.0
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: 'link'" 

rosservice call /gazebo/set_model_state "model_state:
  model_name: 'uavA1'
  pose:
    position:
      x: 20.0
      y: 15.0
      z: 0
    orientation:
      x: 0.0
      y: 0.0
      z: 1.0
      w: 0.0
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: 'link'" 

rosservice call /gazebo/set_model_state "model_state:
  model_name: 'uavA2'
  pose:
    position:
      x: 10.0
      y: 0.0
      z: 0
    orientation:
      x: 0.0
      y: 0.0
      z: 1.0
      w: 0.0
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: 'link'" 

rosservice call /gazebo/set_model_state "model_state:
  model_name: 'uavA3'
  pose:
    position:
      x: 20.0
      y: -15.0
      z: 0
    orientation:
      x: 0.0
      y: 0.0
      z: 1.0
      w: 0.0
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: 'link'" 

rosservice call /gazebo/set_model_state "model_state:
  model_name: 'carB1'
  pose:
    position:
      x: -10.0
      y: 9.0
      z: 0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: 'link'" 

rosservice call /gazebo/set_model_state "model_state:
  model_name: 'carB2'
  pose:
    position:
      x: -10.0
      y: -9.0
      z: 0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: 'link'" 

rosservice call /gazebo/set_model_state "model_state:
  model_name: 'carB3'
  pose:
    position:
      x: -20.0
      y: 0.0
      z: 0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: 'link'" 

rosservice call /gazebo/set_model_state "model_state:
  model_name: 'uavB1'
  pose:
    position:
      x: -20
      y: 15
      z: 0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: 'link'" 
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'uavB2'
  pose:
    position:
      x: -10
      y: 0
      z: 0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: 'link'" 
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'uavB3'
  pose:
    position:
      x: -20
      y: -15
      z: 0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: 'link'" 

