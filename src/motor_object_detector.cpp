#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <rosie_object_detector/ObjectClassify.h>
#include <rosie_object_detector/RAS_Evidence.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include <string>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <boost/filesystem.hpp>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#define GetCurrentDir getcwd


using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";
float robot_x_pos, robot_y_pos, robot_angle;

  void odomCallback(const nav_msgs::Odometry msg){
	robot_x_pos = msg.pose.pose.position.x;
	robot_y_pos = msg.pose.pose.position.y;
	robot_angle = msg.pose.pose.orientation.z;
  }

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub = nh_.subscribe("/odom",100,odomCallback);
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber image_depth;
  image_transport::Publisher image_pub_;
  ros::Publisher vis_pub = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  ros::Publisher vis_pub_bat = nh_.advertise<visualization_msgs::Marker>( "visualization_marker_battery", 0 );
  ros::Publisher evidence_pub = nh_.advertise<rosie_object_detector::RAS_Evidence>("/evidence", 0);
  ros::ServiceClient client = nh_.serviceClient<rosie_object_detector::ObjectClassify>("do_something_with_image");
  rosie_object_detector::ObjectClassify srv;
  visualization_msgs::Marker marker;
  cv::Point p;
  cv::Point p_bat;
  cv::Point p_bat_crop;
  float x_position_object, x_position_bat;
  float y_position_object, y_position_bat;
  float max_dist;
  int depthindex;
  bool show_object_detection_threshold_image;
  bool show_battery_detection_threshold_image, show_battery_detection_threshold_image_depth;
  bool print_center_pixel_hue, print_battery_pixel;
  bool object_detected_prompt, object_detected, object_dissapeared_prompt;
  bool battery_detected_prompt, battery_detected, battery_dissapeared_prompt;
  bool print_color, print_object_coords, print_battery_coords;
  int x_factor, x_offset, y_offset, x_factor_bat, x_offset_bat, y_offset_bat;
  double y_factor, y_factor_bat;
  double battery_factor;
  int detector_edge_padding;
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
  int erosion_size, erosion_size_bat;
  int dilation_elem;
  int dilation_size, dilation_size_bat;
  int color;
  int color_index;
  int current_system;
  int derivative_threshold;
  Mat erosion_dst;
  Mat dilation_dst;
  cv::Mat OriginalImage;
  cv::Mat ThreshImage;
  cv::Mat HSVImage;
  bool check_all_colors;
  int color_index_detected;
  double sleep_duration;
  double lifetime_rviz;
  sensor_msgs::Image msg1;
  std::string classifier_decision;
  cv::Mat Dy;
  int wall_detection_ypixel;

public:
  ImageConverter()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1, &ImageConverter::imageCb, this);
	image_sub_ = it_.subscribe("/camera/depth/image_rect", 1, &ImageConverter::imageCbdepth, this);
	p.x = 0;
    p.y = 0;
	robot_x_pos = 0.0;
	robot_y_pos = 0.0;
	robot_angle = 0.0;
    detector_edge_padding = 5;
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

  void Erosion( int, void*, int type )
  {
    int erosion_type;
    if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
    else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
    else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

    /// Apply the erosion operation
	if (type == 0) {
		Mat element = getStructuringElement( erosion_type,
                                         Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                         Point( erosion_size, erosion_size ) );
    	erode( ThreshImage, erosion_dst, element );
	}
	else {
		Mat element = getStructuringElement( erosion_type,
                                         Size( 2*erosion_size_bat + 1, 2*erosion_size_bat+1 ),
                                         Point( erosion_size_bat, erosion_size_bat ) );
		erode( Dy, Dy, element );
	}
  }

  void Dilation( int, void* , int type)
  {
    int dilation_type;
    if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
    else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
    else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

    if (type == 0){
    Mat element = getStructuringElement( dilation_type,
                                         Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                         Point( dilation_size, dilation_size ) );
    /// Apply the dilation operation
    dilate( erosion_dst, dilation_dst, element );
    }
    else{
    Mat element = getStructuringElement( dilation_type,
                                         Size( 2*dilation_size_bat + 1, 2*dilation_size_bat+1 ),
                                         Point( dilation_size_bat, dilation_size_bat ) );
    /// Apply the dilation operation
    dilate( Dy, Dy, element );
    }
  }

   template<typename T> 
   std::string toString( const T& ao_Obj )
    {
      std::stringstream lo_stream;

      lo_stream << ao_Obj;

      return lo_stream.str();
    }


  void color_selector(){
      nh_.getParam("/sat_low", sat_low);
      nh_.getParam("/sat_high", sat_high);
      nh_.getParam("/value_low", value_low);
      nh_.getParam("/value_high", value_high);
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
    nh_.getParam("/lifetime_rviz", lifetime_rviz);
    marker.header.frame_id = "camera_depth_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = color_index;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(lifetime_rviz);
    if (color_index != 7){
	    if (object_visible){
		marker.pose.position.x = x_position_object;
	    }
	    else {
		marker.pose.position.x = 1e5;
	    }
    marker.pose.position.y = y_position_object;
    }
    else{
	    marker.pose.position.x = x_position_bat;
	    marker.pose.position.y = y_position_bat;
    }
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

	//ROS_INFO()

    if (color_index == 7 && x_position_bat < max_dist){
	vis_pub_bat.publish( marker );
    }
    else if (color_index != 7 && x_position_object < max_dist){
	vis_pub.publish( marker );
    }

    //if (x_position_object < max_dist){
    //    if (color_index == 7){
    //        vis_pub_bat.publish( marker );
    //    }
    //    else {
    //        vis_pub.publish( marker );
    //    }
    //}
      
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
      Erosion(0,0,0);
      Dilation(0,0,0);
      cv::Moments m = moments(dilation_dst,true);
      p.x = m.m10/m.m00;
      p.y = m.m01/m.m00;

      if (p.y > 0){
	  if (color_index == 7) {
	      x_position_object = battery_factor*(x_factor/p.y - 10)/double(100);
	      y_position_object = -1*battery_factor*y_factor*(p.x - 320)/double(600)*x_position_object;
	  }
	  else{
	      x_position_object = (x_factor/p.y - x_offset)/double(100);
	      y_position_object = -1*y_factor*(p.x - y_offset)/double(600)*x_position_object;
	  }

      }
	  
	  if (print_object_coords) {
	  	ROS_INFO("x: %f, y: %f", x_position_object, y_position_object);
	  }

  // Normal object
      if (color_index != 7)
      {
          if (p.x >= detector_edge_padding && p.x <= 640 - detector_edge_padding && p.y >= detector_edge_padding && p.y <= 480 - detector_edge_padding) {
              object_detected = true;
              if (object_dissapeared_prompt) {
                  object_detected_prompt = false;
                  object_dissapeared_prompt = false;
              }
              if (!object_detected_prompt && x_position_object < max_dist){
                  object_detected_prompt = true;
		  static int colored_object_count = 0;
                  ROS_INFO("Object with color %d detected!", color_index);
			
                  // Check classification here
                  // Write to the topic as specified in MS3 here
                  // Use speaker to say what robot sees here

		  colored_object_count = colored_object_count + 2;

		  std::cerr<<current_system<<std::endl;
	          ROS_ASSERT(cv::imwrite(std::string("/home/") + std::string("ras") + toString(current_system) + std::string("/catkin_ws/src/rosie/rosie_object_detector/src/CameraCapture/camera_capture_") + toString(colored_object_count) + std::string(".jpg"), OriginalImage));
		  std_msgs::Int32 number;
		  //number.data = 1;
                  rosie_object_detector::ObjectClassify srv;
                  srv.request.img_number.data = colored_object_count;
		  srv.request.color_ind.data = color_index;
                  if (client.call(srv))
  			{                                
    				ROS_INFO("Successful");
				ROS_INFO("Publishing evidence");
				rosie_object_detector::RAS_Evidence evid;
				evid.stamp = ros::Time::now();
				evid.group_number = 5;
				evid.image_evidence = msg1;
				evid.object_id = srv.response.decision.data;
				evid.object_id_int.data = srv.response.decision_int.data;
				geometry_msgs::Vector3 position_to_send;
                                position_to_send.x = robot_x_pos + x_position_object*cos(robot_angle) - y_position_object*sin(robot_angle); // Not tested if correct
                                position_to_send.y = robot_y_pos + x_position_object*sin(robot_angle) + y_position_object*cos(robot_angle); // Not tested if correct
				evid.object_location = position_to_send;
				evidence_pub.publish(evid); 
  			}
  		  else
  			{
    				ROS_ERROR("Failed to call service do_something_with_image");
  			}
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
              //publish_test(false);
          }
      }
      else {
          if (p.x >= detector_edge_padding && p.x <= 640 - detector_edge_padding && p.y >= detector_edge_padding && p.y <= 480 - detector_edge_padding) {
              battery_detected = true;
              if (battery_dissapeared_prompt) {
                  battery_detected_prompt = false;
                  battery_dissapeared_prompt = false;
              }
              if (!battery_detected_prompt && x_position_object < max_dist){
                  battery_detected_prompt = true;
                  ROS_INFO("Battery detected!");
		  static int battery_object_count = 1;
                  battery_object_count = battery_object_count + 2;
		  ROS_ASSERT(cv::imwrite(std::string("/home/") + std::string("ras") + toString(current_system) + std::string("/catkin_ws/src/rosie/rosie_object_detector/src/CameraCapture/camera_capture_") + toString(battery_object_count) + std::string(".jpg"), OriginalImage));

		  std_msgs::Int32 number;
                  rosie_object_detector::ObjectClassify srv;
                  srv.request.img_number.data = battery_object_count;
		  srv.request.color_ind.data = color_index;
                  if (client.call(srv))
  			{
    				ROS_INFO("Successful");
					ROS_INFO("Publishing evidence");
				rosie_object_detector::RAS_Evidence evid;
				evid.stamp = ros::Time::now();
				evid.group_number = 5;
				evid.image_evidence = msg1;
				evid.object_id = srv.response.decision.data;
				evid.object_id_int.data = srv.response.decision_int.data;
				geometry_msgs::Vector3 position_to_send;
                position_to_send.x = robot_x_pos + x_position_object*cos(robot_angle) - y_position_object*sin(robot_angle); // Not tested if correct
                position_to_send.y = robot_y_pos + x_position_object*sin(robot_angle) + y_position_object*cos(robot_angle); // Not tested if correct
				evid.object_location = position_to_send;
				evidence_pub.publish(evid);
  			}
  		  else
  			{
    				ROS_ERROR("Failed to call service do_something_with_image");
  			}
                  // Check classification here
                  // Write to the topic as specified in MS3 here
                  // Use speaker to say what robot sees here
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
              //publish_test(false);
          }
      }

      circle(dilation_dst, p, 5, cv::Scalar(128,0,0), -1);
      if (show_object_detection_threshold_image && color_index != 7){
          //cv::imshow("Image with center",ThreshImage);
          cv::imshow("Image with center",dilation_dst);
      }
      else if (show_battery_detection_threshold_image && color_index == 7){
          //cv::imshow("Image with center",ThreshImage);
          cv::imshow("Image with center",dilation_dst);
      }
      cv::waitKey(3);

      // Output modified video stream
      //image_pub_.publish(cv_ptr->toImageMsg());
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    nh_.getParam("/max_dist", max_dist);
    nh_.getParam("/sat_low", sat_low);
    nh_.getParam("/sat_high", sat_high);
    nh_.getParam("/hue", hue);
    nh_.getParam("/hue_interval", hue_interval);
    nh_.getParam("/value_low", value_low);
    nh_.getParam("/value_high", value_high);
    nh_.getParam("/show_object_detection_threshold_image", show_object_detection_threshold_image);
    nh_.getParam("/show_battery_detection_threshold_image", show_battery_detection_threshold_image);
    nh_.getParam("/print_center_pixel_hue", print_center_pixel_hue);
    nh_.getParam("/print_color", print_color);
    nh_.getParam("/x_factor", x_factor);
    nh_.getParam("/y_factor", y_factor);
    nh_.getParam("/x_offset", x_offset);
    nh_.getParam("/y_offset", y_offset);
    nh_.getParam("/battery_factor", battery_factor);
    nh_.getParam("/erosion_size", erosion_size);
    nh_.getParam("/dilation_size", dilation_size);
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
    nh_.getParam("/current_system", current_system);
    nh_.getParam("/detector_edge_padding", detector_edge_padding);
	nh_.getParam("/print_object_coords", print_object_coords);
    msg1 = *msg;

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
    OriginalImage = cv_ptr->image;

    if (object_detected && check_all_colors) {
        color_index = color_index_detected;
        color_checker();
    }
    else if (check_all_colors){
        color_index = 0;
        while (color_index <= 6 && !object_detected){
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
    //color_index = 7;
    //color_checker();
    ros::Duration(sleep_duration).sleep();
  }

  void imageCbdepth(const sensor_msgs::ImageConstPtr& msg) {
	nh_.getParam("/derivative_threshold", derivative_threshold);
    	nh_.getParam("/erosion_size_bat", erosion_size_bat);
	nh_.getParam("/dilation_size_bat", dilation_size_bat);
	nh_.getParam("/show_battery_detection_threshold_image_depth", show_battery_detection_threshold_image_depth);
	nh_.getParam("/print_battery_pixel", print_battery_pixel);
	nh_.getParam("/wall_detection_ypixel", wall_detection_ypixel);
	nh_.getParam("/x_factor_bat", x_factor_bat);
	nh_.getParam("/x_offset_bat", x_offset_bat);
	nh_.getParam("/y_factor_bat", y_factor_bat);
	nh_.getParam("/y_offset_bat", y_offset_bat);
	nh_.getParam("/print_battery_coords", print_battery_coords);
	nh_.getParam("/max_dist", max_dist);

	int scale = 1;
	int delta = 0;
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	cv::Sobel(cv_ptr->image, Dy, CV_32F, 0, 1, 5);
    	cv::GaussianBlur(Dy, Dy, Size( 0, 0), 2, 0);
	cv::threshold(Dy, Dy, derivative_threshold, 1, 0);
	Erosion(0,0,1);
	Dilation(0,0,1);
	//cv::normalize(Dy, Dy, 0, 255, cv::NORM_MINMAX);

	cv::Moments m_bat = moments(Dy,true);
	p_bat.x = m_bat.m10/m_bat.m00;
	p_bat.y = m_bat.m01/m_bat.m00;
	
	cv::Rect roi;
	roi.width = 60;
	roi.height = wall_detection_ypixel;
	if (p_bat.x < roi.width/2) {
		roi.x = 0;
	}
	else if (p_bat.x > 640 - roi.width/2){
		roi.x = 640;
	}
	else {
		roi.x = p_bat.x - roi.width/2;	
	}
	roi.y = 0;

	cv::Mat Dy_crop = Dy(roi);

	cv::Moments m_bat_crop = moments(Dy_crop,true);
	p_bat_crop.x = m_bat_crop.m10/m_bat_crop.m00;
	p_bat_crop.y = m_bat_crop.m01/m_bat_crop.m00;
	//ROS_INFO("x: %d, y: %d", p_bat_crop.x, p_bat_crop.y);

	if (print_battery_pixel) {
		ROS_INFO("x: %d, y: %d", p_bat.x, p_bat.y);
	}

	if (p_bat.x > 0 && p_bat.y > 0 && p_bat_crop.x < 0){
		//ROS_INFO("Battery detected (by depth data)");
		
		x_position_bat = (x_factor_bat/p_bat.y - x_offset_bat)/double(100);
		y_position_bat = -1*y_factor_bat*(p_bat.x - y_offset_bat)/double(600)*x_position_bat;
		
		if (print_battery_coords){
			ROS_INFO("coords, x: %f, y: %f", x_position_bat, y_position_bat);
		}		

		int color_index_tmp = color_index;
		color_index = 7;		
		publish_test(true);
		color_index = color_index_tmp;

	}

	if (show_battery_detection_threshold_image_depth){
		//circle(Dy, p_bat, 5, cv::Scalar(128,128,0), -1);
		cv::imshow("Depth derivative", Dy);
		cv::waitKey(3);
	}
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
