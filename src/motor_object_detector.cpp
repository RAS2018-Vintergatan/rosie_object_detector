#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber image_sub_depth;
  image_transport::Publisher image_pub_;
  cv::Point p;
  int depthindex;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1, &ImageConverter::imageCb, this);
    image_sub_depth = it_.subscribe("/camera/depth/image", 1, &ImageConverter::imageCb_depth, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    p.x = 0;
    p.y = 0;
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb_depth(const sensor_msgs::ImageConstPtr& msg)
  {
      unsigned long depthvalue;
      depthindex = ((msg->width)*(p.y) + p.x)*4; // This is not right yet
      depthvalue = msg->data[depthindex] + msg->data[depthindex + 1]<<8 + msg->data[depthindex + 2]<<16 + msg->data[depthindex + 3]<<24;
      ROS_INFO("The depth at the point %d is %ld", depthindex, depthvalue);
      ROS_INFO("%d", msg->data[depthindex]);
      ROS_INFO("%d", msg->data[depthindex + 1]<<8);
      ROS_INFO("%d", msg->data[depthindex + 2]<<16);
      ROS_INFO("%d", msg->data[depthindex + 3]<<24);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    cv::Mat HSVImage;
    cv::Mat ThreshImage;
    cvtColor(cv_ptr->image,HSVImage,CV_BGR2HSV);
    inRange(HSVImage,cv::Scalar(90,150,100),cv::Scalar(110,255,255),ThreshImage);
    cv::Moments m = moments(ThreshImage,true);
    p.x = m.m10/m.m00;
    p.y = m.m01/m.m00;

    ROS_INFO("Point coordinate %d, %d", p.x, p.y);

    circle(ThreshImage, p, 5, cv::Scalar(128,0,0), -1);
    cv::imshow("Image with center",ThreshImage);

    // Update GUI Window

    //cv::imshow(OPENCV_WINDOW, ThreshImage);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
