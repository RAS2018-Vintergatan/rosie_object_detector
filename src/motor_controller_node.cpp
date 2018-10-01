#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ras_lab1_msgs/PWM.h>
#include <ras_lab1_msgs/Encoders.h>


double t_W1 = 0;
double t_W2 = 0;

double Kp1 = 1.0;
double Kp2 = 1.0;

double Ki1 = 1.0;
double Ki2 = 1.0;

double wlR = 1.0;
double wrR = 1.0;

double wheelSeparation = 1.0;

float motor1PWM;
float motor2PWM;

double errorSum1 = 0;
double errorSum2 = 0;

void setPWM(float motor1, float motor2){
    motor1PWM = motor1;
    motor2PWM = motor2;
}

/*
int32 encoder1;
int32 encoder2;
int32 delta_encoder1;
int32 delta_encoder2;
int32 timestamp;
*/
void encodersCallback(const ras_lab1_msgs::Encoders& msg){
    ROS_INFO("-----------");
    ROS_INFO("E1: %d, E2: %d, dE1: %d, dE2: %d", msg.encoder1,msg.encoder1,msg.delta_encoder1,msg.delta_encoder2);
    
    double est_w1 = ((double)msg.delta_encoder1 * 10.0 * 2.0 * 3.1415)/(360.0);
    double est_w2 = ((double)msg.delta_encoder2 * 10.0 * 2.0 * 3.1415)/(360.0);
    
    double error1 = t_W1-est_w1;
    double error2 = t_W2-est_w2;

    errorSum1 += error1;
    errorSum2 += error2;    

    ROS_INFO("WlR: %f, B: %f, Kp1: %f, Kp2: %f, Ki1: %f, Ki2: %f", wlR, wheelSeparation, Kp1, Kp2, Ki1, Ki2);
    ROS_INFO("TW_M1: %f, W_M1: %f, PWM_M1: %f\nTW_M2: %f, W_M2: %f, PWM_M2: %f",
	 t_W1, est_w1, motor1PWM, t_W2, est_w2, motor2PWM);

    motor1PWM = (motor1PWM + Kp1*error1 + Ki1*errorSum1);
    motor2PWM = (motor2PWM + Kp2*error2 + Ki2*errorSum2);

    motor1PWM = motor1PWM<255?motor1PWM:255;
    motor1PWM = motor1PWM>-255?motor1PWM:-255;
    motor2PWM = motor2PWM<255?motor2PWM:255;
    motor2PWM = motor2PWM>-255?motor2PWM:-255;

    ROS_INFO("-----------");
}

/*
geometry_msgs/Vector3 linear
  float64 x <-- 2D
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z <-- 2D
*/
void controllerCallback(const geometry_msgs::Twist& msg){
    ROS_INFO("LinX: %f, AngZ: %f", 
	msg.linear.x, msg.angular.z);

    double wl = msg.linear.x/wlR;
    double wr = msg.linear.x/wrR;
    wl -= (wheelSeparation*msg.angular.z)/(2.0*wlR);
    wr += (wheelSeparation*msg.angular.z)/(2.0*wrR);

//    double wl = ((msg.linear.x)-(wheelSeparation/2)*msg.angular.z)/wlR;
//    double wr = ((msg.linear.x)+(wheelSeparation/2)*msg.angular.z)/wrR;

    t_W1 = wl;
    t_W2 = wr;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "motor_controller");

    ros::NodeHandle n;

    std::map<std::string,double> m1_pid, m2_pid;
    n.getParam("motor_left_pid", m1_pid);
    n.getParam("motor_right_pid", m2_pid);

    Kp1 = m1_pid["Kp"];
    Ki1 = m1_pid["Ki"];
    Kp2 = m2_pid["Kp"];
    Ki2 = m2_pid["Ki"];

    n.getParam("wheel_left_radius", wlR);
    n.getParam("wheel_right_radius", wrR);

    n.getParam("wheel_separation", wheelSeparation);

    ros::Publisher motor_pub = n.advertise<ras_lab1_msgs::PWM>("/kobuki/pwm",1);
    ros::Subscriber encoder_sub = n.subscribe("/kobuki/encoders", 1, encodersCallback);
    ros::Subscriber controller_sub = n.subscribe("/motor_controller/twist", 1, controllerCallback);

    ros::Rate loop_rate(10);

    while(ros::ok()){

        ras_lab1_msgs::PWM msg;

        msg.PWM1 = (int)motor1PWM;
        msg.PWM2 = (int)motor2PWM;

        motor_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
