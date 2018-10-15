#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>
#include <boost/foreach.hpp>
#include <visualization_msgs/Marker.h>
//#include <math.h>


using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";



class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber image_depth;
  ros::Subscriber image_sub_depth;
  image_transport::Publisher image_pub_;
  ros::Publisher vis_pub = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  visualization_msgs::Marker marker;
  cv::Point p;
  float x_position_object;
  float y_position_object;
  int depthindex;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1, &ImageConverter::imageCb, this);
    image_depth = it_.subscribe("/camera/depth/image_rect", 1, &ImageConverter::DepthImage, this);
    image_sub_depth = nh_.subscribe("/camera/depth/points", 1,  &ImageConverter::imageCb_depth, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    p.x = 0;
    p.y = 0;
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void DepthImage(const sensor_msgs::ImageConstPtr& msg)
  {
      //printf("The encoding is %s", msg->encoding);
      if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      {
         const float *depth_array = reinterpret_cast<const float*>(&(msg->data[0]));
         //printf("The depth of the 0th pixel is:\t %f \n", *depth_array);
      }


  }

  void imageCb_depth(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
      //BOOST_FOREACH (const pcl::PointXYZ& pt, msg->data)
      //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

      int width = msg->width;
      int height = msg->height;

      //printf("\t %d %d \n", width, height);

      pcl::PointCloud<pcl::PointXYZ> input_;
      pcl::fromROSMsg(*msg, input_);
      //pcl::PointXYZ pt = input_[200];
      //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
      int ct = 0;
      BOOST_FOREACH (const pcl::PointXYZ& pt, input_.points)
        //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
              ct++;
      //printf("\t %d \n", ct);
  }

  void pixelTo3DPoint(const sensor_msgs::PointCloud2ConstPtr& pCloud, const int u, const int v)
  {
    // get width and height of 2D point cloud data
    int width = pCloud->width;
    int height = pCloud->height;

    // Convert from u (column / width), v (row/height) to position in array
    // where X,Y,Z data starts
    int arrayPosition = v*pCloud->row_step + u*pCloud->point_step;

    // compute position in array where x,y,z data start
    int arrayPosX = arrayPosition + pCloud->fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + pCloud->fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + pCloud->fields[2].offset; // Z has an offset of 8

    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;

    memcpy(&X, &pCloud->data[arrayPosX], sizeof(float));
    memcpy(&Y, &pCloud->data[arrayPosY], sizeof(float));
    memcpy(&Z, &pCloud->data[arrayPosZ], sizeof(float));

    //ROS_INFO("%f, %f, %f", X, Y, Z);

  //  p.x = X;
  //  p.y = Y;
  //  p.z = Z;

  }

  void publish_test()
  {
      marker.header.frame_id = "camera_depth_frame";
      marker.header.stamp = ros::Time();
      marker.ns = "my_namespace";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = x_position_object;
      marker.pose.position.y = y_position_object;
      marker.pose.position.z = 0.05;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      //only if using a MESH_RESOURCE marker type:
      marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
      vis_pub.publish( marker );
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
    inRange(HSVImage,cv::Scalar(90,150,100),cv::Scalar(100,255,255),ThreshImage);
    cv::Moments m = moments(ThreshImage,true);
    p.x = m.m10/m.m00;
    p.y = m.m01/m.m00;

    // Naive x and y position calculations
    x_position_object = (120e2/p.y - 10)/double(100);
    y_position_object = sin(-1*(p.x - 320)/double(600))*x_position_object;
    publish_test();
    //ROS_INFO("The [x, y] coordinate of the object: [%f, %f]", x_position_object, y_position_object);
    // End of naive x and y position calculations

//    ROS_INFO("Point coordinate %d, %d", p.x, p.y);

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
