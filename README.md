# rosie_object_detector

## How to run

## Start the camera

roslaunch realsense_camera sr300_nodelet_rgbd.launch

## Load params

object_detector_params.yaml

## Start the detector

rosrun rosie_object_detector rosie_object_detector

## For rviz 
In rviz add a marker and use topic "visualization_marker" (should be the standard).
