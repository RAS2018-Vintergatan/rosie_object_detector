// First start the camera

roslaunch realsense_camera sr300_nodelet_rgbd.launch

// Secondly start the detector

rosrun rosie_object_detector rosie_object_detector

// Don't look here guys
// Good site for multiple blobs/centers

https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
