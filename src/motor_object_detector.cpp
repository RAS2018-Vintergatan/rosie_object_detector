#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>


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
  ros::Publisher vis_pub_bat = nh_.advertise<visualization_msgs::Marker>( "visualization_marker_battery", 0 );
  visualization_msgs::Marker marker;
  cv::Point p;
  float x_position_object;
  float y_position_object;
  int depthindex;
  bool show_object_detection_threshold_image;
  bool print_center_pixel_hue;
  bool object_detected_prompt, object_detected, object_dissapeared_prompt;
  bool battery_detected_prompt, battery_detected, battery_dissapeared_prompt;
  bool print_color;
  int x_factor;
  int hue;
  int hue_interval;
  int sat_low;
  int sat_high;
  int value_low;
  int value_high;
  int color_interval;
  int hsv_red, hsv_red_interval;
  int hsv_orange, hsv_orange_interval;
  int hsv_yellow, hsv_yellow_interval, yellow_sat_low, yellow_value_low;
  int hsv_light_green, hsv_light_green_interval;
  int hsv_dark_green, hsv_dark_green_interval;
  int hsv_blue, hsv_blue_interval;
  int hsv_purple, hsv_purple_interval;
  int hsv_battery_hue, hsv_battery_hue_interval, hsv_battery_sat_low, hsv_battery_sat_high, hsv_battery_value_low, hsv_battery_value_high;
  int erosion_elem;
  int erosion_size;
  int dilation_elem;
  int dilation_size;
  int color;
  int color_index;
  Mat erosion_dst;
  Mat dilation_dst;
  cv::Mat ThreshImage;
  cv::Mat HSVImage;
  bool check_all_colors;
  int color_index_detected;
  double sleep_duration;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1, &ImageConverter::imageCb, this);
    //image_depth = it_.subscribe("/camera/depth/image_rect", 1, &ImageConverter::DepthImage, this);
    //image_sub_depth = nh_.subscribe("/camera/depth/points", 1,  &ImageConverter::imageCb_depth, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    p.x = 0;
    p.y = 0;
    erosion_elem = 0;
    dilation_elem = 0;
    object_detected_prompt = false;
    object_detected = false;
    object_dissapeared_prompt = false;
    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void Erosion( int, void* )
  {
    int erosion_type;
    if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
    else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
    else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

    Mat element = getStructuringElement( erosion_type,
                                         Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                         Point( erosion_size, erosion_size ) );

    /// Apply the erosion operation
    erode( ThreshImage, erosion_dst, element );
  }

  void Dilation( int, void* )
  {
    int dilation_type;
    if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
    else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
    else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

    Mat element = getStructuringElement( dilation_type,
                                         Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                         Point( dilation_size, dilation_size ) );
    /// Apply the dilation operation
    dilate( erosion_dst, dilation_dst, element );
  }

//  void pixel_color(){
//      Vec3b intensity = HSVImage.at<Vec3b>(p.y,p.x);
//      uchar hue = intensity.val[0];

//      if (hue < hsv_red + color_interval && hue > hsv_red - color_interval) {
//          color = 1;
//      }
//      else if (hue < hsv_orange + color_interval && hue > hsv_orange - color_interval) {
//          color = 2;
//      }
//      else if (hue < hsv_yellow + color_interval && hue > hsv_yellow - color_interval) {
//          color = 3;
//      }
//      else if (hue < hsv_green + color_interval && hue > hsv_green - color_interval) {
//          color = 4;
//      }
//      else if (hue < hsv_blue + color_interval && hue > hsv_blue - color_interval) {
//          color = 5;
//      }
//      else if (hue < hsv_purple + color_interval && hue > hsv_purple - color_interval) {
//          color = 6;
//      }
//      else {
//          color = 0;
//          ROS_WARN("No color detected. Might have to adjust sat and value and/or hsv color values and color interval in object_detector_params.yaml");
//      }
//      if (print_color){
//      ROS_INFO("Color: %d", color);
//      }
//  }

  void color_selector(){
      if (color_index == 0) {
          hue = hsv_red;
          hue_interval = hsv_red_interval;
      }
      else if (color_index == 1){
          hue = hsv_orange;
          hue_interval = hsv_orange_interval;
          sat_low = yellow_sat_low;
          value_low = yellow_value_low;
      }
      else if (color_index == 2){
          hue = hsv_yellow;
          hue_interval = hsv_yellow_interval;
          sat_low = yellow_sat_low;
          value_low = yellow_value_low;
      }
      else if (color_index == 3){
          hue = hsv_light_green;
          hue_interval = hsv_light_green_interval;
      }
      else if (color_index == 4){
          hue = hsv_dark_green;
          hue_interval = hsv_dark_green_interval;
      }
      else if (color_index == 5){
          hue = hsv_blue;
          hue_interval = hsv_blue_interval;
      }
      else if (color_index == 6){
          hue = hsv_purple;
          hue_interval = hsv_purple_interval;
      }
      else if (color_index == 7){
          hue = hsv_battery_hue;
          hue_interval = hsv_battery_hue_interval;
          sat_low = hsv_battery_sat_low;
          sat_high = hsv_battery_sat_high;
          value_low = hsv_battery_value_low;
          value_high = hsv_battery_value_high;
      }
  }

  void publish_test(bool object_visible)
  {
    marker.header.frame_id = "camera_depth_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    if (object_visible){
        marker.pose.position.x = x_position_object;
    }
    else {
        marker.pose.position.x = 1e5;
    }
    marker.pose.position.y = y_position_object;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    if (color_index == 0){
        marker.color.r = 1.0;
    }
    else if (color_index == 1){
        marker.color.r = 1.0;
        marker.color.g = 0.6;
    }
    else if (color_index == 2) {
        marker.color.r = 1.0;
        marker.color.g = 0.9;
        marker.color.b = 0.1;
    }
    else if (color_index == 3 || color_index == 4){
        marker.color.g = 1.0;
    }
    else if (color_index == 5){
        marker.color.b = 1.0;
    }
    else if (color_index == 6){
        marker.color.r = 0.6;
        marker.color.b = 1.0;
    }
    else if (color_index == 7){
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }

    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    if (color_index == 7){
        vis_pub_bat.publish( marker );
    }
    else {
        vis_pub.publish( marker );
    }
      
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.06, 0, 0) );
    tf::Quaternion qtf;
    qtf.setRPY(0, 0, 0);
    transform.setRotation( qtf );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", marker.header.frame_id));
    }

  void color_checker(){
      color_selector();
      inRange(HSVImage,cv::Scalar(hue - hue_interval,sat_low,value_low),cv::Scalar(hue + hue_interval,sat_high,value_high),ThreshImage);
      Erosion(0,0);
      Dilation(0,0);
      cv::Moments m = moments(dilation_dst,true);
      p.x = m.m10/m.m00;
      p.y = m.m01/m.m00;

      x_position_object = (x_factor/p.y - 10)/double(100);
      y_position_object = sin(-1*(p.x - 320)/double(600))*x_position_object;

      // Normal object
      if (color_index != 7)
      {
          if (p.x > -1 && p.y > -1) {
              object_detected = true;
              if (object_dissapeared_prompt) {
                  object_detected_prompt = false;
                  object_dissapeared_prompt = false;
              }
              if (!object_detected_prompt){
                  object_detected_prompt = true;
                  ROS_INFO("Object detected!");
              }
          }
          else {
              object_detected = false;
          }
          if (object_detected) {
              if (print_center_pixel_hue){
                  Vec3b intensity = HSVImage.at<Vec3b>(p.y,p.x);
                  uchar hue = intensity.val[0];
                  uchar sat = intensity.val[1];
                  uchar value = intensity.val[2];
                  ROS_INFO("Center pixel hue: %d, sat: %d, value: %d", hue, sat, value);
              }
              //pixel_color();
              publish_test(true);
          }
          else {
              if (object_detected_prompt && !object_dissapeared_prompt){
                  object_dissapeared_prompt = true;
                  ROS_INFO("Object dissapeared!");
              }
              publish_test(false);
          }
      }
      else {
          if (p.x > -1 && p.y > -1) {
              battery_detected = true;
              if (battery_dissapeared_prompt) {
                  battery_detected_prompt = false;
                  battery_dissapeared_prompt = false;
              }
              if (!battery_detected_prompt){
                  battery_detected_prompt = true;
                  ROS_INFO("Battery detected!");
              }
          }
          else {
              battery_detected = false;
          }
          if (battery_detected) {
              if (print_center_pixel_hue){
                  Vec3b intensity = HSVImage.at<Vec3b>(p.y,p.x);
                  uchar hue = intensity.val[0];
                  uchar sat = intensity.val[1];
                  uchar value = intensity.val[2];
                  ROS_INFO("Center pixel hue: %d, sat: %d, value: %d", hue, sat, value);
              }
              //pixel_color();
              publish_test(true);
          }
          else {
              if (battery_detected_prompt && !battery_dissapeared_prompt){
                  battery_dissapeared_prompt = true;
                  ROS_INFO("Battery dissapeared!");
              }
              publish_test(false);
          }
      }

      circle(dilation_dst, p, 5, cv::Scalar(128,0,0), -1);
      if (show_object_detection_threshold_image){
          //cv::imshow("Image with center",ThreshImage);
          cv::imshow("Image with center",dilation_dst);
      }
      cv::waitKey(3);

      // Output modified video stream
      //image_pub_.publish(cv_ptr->toImageMsg());
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    nh_.getParam("/sat_low", sat_low);
    nh_.getParam("/sat_high", sat_high);
    nh_.getParam("/hue", hue);
    nh_.getParam("/hue_interval", hue_interval);
    nh_.getParam("/value_low", value_low);
    nh_.getParam("/value_high", value_high);
    nh_.getParam("/show_object_detection_threshold_image", show_object_detection_threshold_image);
    nh_.getParam("/print_center_pixel_hue", print_center_pixel_hue);
    nh_.getParam("/print_color", print_color);
    nh_.getParam("/x_factor", x_factor);
    nh_.getParam("erosion_size", erosion_size);
    nh_.getParam("dilation_size", dilation_size);
    nh_.getParam("/color_interval", color_interval);
    nh_.getParam("/hsv_red", hsv_red);
    nh_.getParam("/hsv_orange", hsv_orange);
    nh_.getParam("/hsv_yellow", hsv_yellow);
    nh_.getParam("/hsv_light_green", hsv_light_green);
    nh_.getParam("/hsv_dark_green", hsv_dark_green);
    nh_.getParam("/hsv_blue", hsv_blue);
    nh_.getParam("/hsv_purple", hsv_purple);
    nh_.getParam("/hsv_red_interval", hsv_red_interval);
    nh_.getParam("/hsv_orange_interval", hsv_orange_interval);
    nh_.getParam("/hsv_yellow_interval", hsv_yellow_interval);
    nh_.getParam("/hsv_light_green_interval", hsv_light_green_interval);
    nh_.getParam("/hsv_dark_green_interval", hsv_dark_green_interval);
    nh_.getParam("/hsv_blue_interval", hsv_blue_interval);
    nh_.getParam("/hsv_purple_interval", hsv_purple_interval);
    nh_.getParam("/yellow_sat_low", yellow_sat_low);
    nh_.getParam("/yellow_value_low", yellow_value_low);
    nh_.getParam("/color_index", color_index);
    nh_.getParam("/check_all_colors", check_all_colors);

    nh_.getParam("/hsv_battery_hue", hsv_battery_hue);
    nh_.getParam("/hsv_battery_hue_interval", hsv_battery_hue_interval);
    nh_.getParam("/hsv_battery_sat_low", hsv_battery_sat_low);
    nh_.getParam("/hsv_battery_sat_high", hsv_battery_sat_high);
    nh_.getParam("/hsv_battery_value_low", hsv_battery_value_low);
    nh_.getParam("/hsv_battery_value_high", hsv_battery_value_high);
    nh_.getParam("/sleep_duration_object_detector", sleep_duration);


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
    cvtColor(cv_ptr->image,HSVImage,CV_BGR2HSV);

    if (object_detected && check_all_colors) {
        color_index = color_index_detected;
        color_checker();
    }
    else if (check_all_colors){
        color_index = 0;
        while (color_index < 6 && !object_detected){
            color_checker();
            if (object_detected){
                color_index_detected = color_index;
            }
            ros::Duration(sleep_duration).sleep();
            color_index++;
        }
     }
    else {
        color_checker();
    }
    // Battery check
    color_index = 7;
    color_checker();
    ros::Duration(sleep_duration).sleep();
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");

  ImageConverter ic;
  ros::spin();
  return 0;
}
