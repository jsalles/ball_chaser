#include "ball_chaser/DriveToTarget.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

class DriveBot {
private:
  ros::Publisher motor_command_publisher;

  bool handle_drive_request(ball_chaser::DriveToTarget::Request &req,
                            ball_chaser::DriveToTarget::Response &res) {
    ROS_INFO("Received message to move to linear_x: %.2f, angular_z: %.2f",
             req.linear_x, req.angular_z);

    geometry_msgs::Twist motor_command;
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    motor_command_publisher.publish(motor_command);

    res.msg_feedback = "linear_x: " + std::to_string(req.linear_x) +
                       ", angular_z: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
  }

public:
  DriveBot() {
    ros::NodeHandle n;
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::ServiceServer serviceServer = n.advertiseService(
        "/ball_chaser/command_robot", &DriveBot::handle_drive_request, this);
    ROS_INFO("Ready to accept movement commands");
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "drive_bot");

  DriveBot drive_bot;

  ros::spin();

  return 0;
}
