// First start the camera

roslaunch realsense_camera sr300_nodelet_rgbd.launch
(ignore errors)

// Secondly start the detector

rosrun rosie_object_detector rosie_object_detector

// Thirdly for rviz
In rviz add a marker and use topic "visualization_marker" (should be the standard). We are right now using frame "camera_depth_frame", 
we have to change that later to the correct frame.










// Don't look here guys
// Good site for multiple blobs/centers

https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/

Refer to these links:

http://wiki.ros.org/depth_image_proc
https://answers.ros.org/question/246066/how-can-i-get-object-distance-using-cameradepthimage_raw/
https://answers.ros.org/question/9510/disparity-image-publishing/
