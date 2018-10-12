#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>




void RGB_Callback(const sensor_msgs::Image& msg){

    //ROS_INFO("The image height is %d ", msg.height);

    cv_bridge::CvImagePtr cv_ptr;
    Mat HSVImage;
    Mat ThreshImage;
    // transform ROS image into OpenCV image
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    //Transform the colors into HSV
    cvtColor(cv_ptr->image,HSVImage,CV_BGR2HSV);
    //for blue
    inRange(HSVImage,Scalar(80,90,70),Scalar(110,125,200),ThreshImage);

}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_object_detector");
    ros::NodeHandle n;

    //ros::Publisher motorLeft_pub = n.advertise<std_msgs::Float32>("/motorLeft/cmd_vel",1);
    ros::Subscriber RGB_sub = n.subscribe("/camera/rgb/image_rect_color", 100, RGB_Callback);

    ros::Rate loop_rate(10);

    while(ros::ok()){

        ros::spinOnce();
        loop_rate.sleep();
    }
}

