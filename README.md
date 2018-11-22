# rosie_object_detector

## How to run

# Start the camera

roslaunch realsense_camera sr300_nodelet_rgbd.launch

# Start the classifier

rosrun rosie_object_detector classify_image_server 

# Load params

object_detector_params.yaml

# Start the detector

rosrun rosie_object_detector rosie_object_detector

# Remember to do a rosbag of /evidence

rosbag record /evidence

## Misc

# For rviz 
In rviz, for object marker, add a marker and use topic "visualization_marker" (should be the standard).
For a battery marker, add a marker and use topic "visualization_marker_battery"

# Good to know

No object is detected if there is nothing being published to the topic/s.
The markers have the color of the detected object.

color_index (in the code) and id (in the marker msg):
0 = red
1 = orange
2 = yellow
3 = light green
4 = dark green
5 = blue
6 = purple
7 = battery

Check params file for: 
Changing HSV intervals.
Check for all colors or just one.
Show window with thresholded image.
Calibration constants for position estimation.
etc.
