#include "ball_chaser/DriveToTarget.h"
#include "ros/ros.h"
#include <sensor_msgs/Image.h>

class ProcessImage {
private:
  ros::ServiceClient client;
  ros::Subscriber sub;
  bool is_chasing;

  void process_image_callback(const sensor_msgs::Image img);
  void drive_robot(float lin_x, float ang_z);
  bool is_white_pixel(const sensor_msgs::Image &img, int pos);

public:
  ProcessImage(ros::NodeHandle *n);
};

ProcessImage::ProcessImage(ros::NodeHandle *n) {
  client = n->serviceClient<ball_chaser::DriveToTarget>(
      "/ball_chaser/command_robot");

  sub = n->subscribe("/camera/rgb/image_raw", 10,
                     &ProcessImage::process_image_callback, this);

  is_chasing = false;
}

void ProcessImage::drive_robot(float lin_x, float ang_z) {
  ball_chaser::DriveToTarget req;
  req.request.linear_x = lin_x;
  req.request.angular_z = ang_z;
  client.call(req);
}

bool ProcessImage::is_white_pixel(const sensor_msgs::Image &img, int pos) {
  int white_pixel = 255;
  return img.data[pos] == white_pixel && img.data[pos + 1] == white_pixel &&
         img.data[pos + 2] == white_pixel;
}

void ProcessImage::process_image_callback(const sensor_msgs::Image img) {
  float chase_velocity = 0.5f;

  for (int i = 0; i < img.height * img.step; i += 3) {
    if (is_white_pixel(img, i)) {
      is_chasing = true;
      int current_col = i % img.step;
      ROS_INFO("Ball found at %d column", current_col);
      if (current_col < img.step / 3) {
        ROS_INFO("Chasing left");
        drive_robot(0, chase_velocity);
      } else if (current_col < 2 * img.step / 3) {
        ROS_INFO("Chasing forward");
        drive_robot(chase_velocity, 0);
      } else {
        ROS_INFO("Chasing right");
        drive_robot(0, -1 * chase_velocity);
      }

      return;
    }
  }

  if (is_chasing) {
    ROS_INFO("Ball not found. Stopping");
    is_chasing = false;
    drive_robot(0, 0);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  ProcessImage process_image(&n);

  // Handle ROS communication events
  ros::spin();

  return 0;
}
