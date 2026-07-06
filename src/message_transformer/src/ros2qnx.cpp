#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <ctime>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <thread>
#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include "message_transformer/SimpleCMD.h"
#include "message_transformer/ComplexCMD.h"
#include "../include/protocol.h"
#include "../include/sensor_logger.h"

using namespace std;

class ROS2QNX
{
public:
  ROS2QNX(ros::NodeHandle& nh)
  {
    std::string remote_ip;
    int remote_port;
    int local_port;
    bool isdebug;
    nh.param<std::string>("remote_ip", remote_ip, "192.168.1.120");
    nh.param<int>("remote_port", remote_port, 43893);
    nh.param<int>("local_port", local_port, 43894);
    nh.param<bool>("isdebug", isdebug, false);
    nh.param<double>("vel_x_factor", vel_x_factor_, 1.0);
    nh.param<double>("vel_y_factor", vel_y_factor_, 1.0);
    nh.param<double>("vel_yaw_factor", vel_yaw_factor_, 1.0);
    isdebug_ = isdebug;

    fd_ = socket(AF_INET, SOCK_DGRAM,0);
    if(fd_==-1){
      ROS_WARN("scoket create failed!");
    }

    if(local_port > 0){
      struct sockaddr_in addr_local;
      memset(&addr_local, 0, sizeof(addr_local));
      addr_local.sin_family = AF_INET;
      addr_local.sin_port = htons(local_port);
      addr_local.sin_addr.s_addr = htonl(INADDR_ANY);
      int bind_ret = bind(fd_, (struct sockaddr *)&addr_local, sizeof(addr_local));
      if(bind_ret < 0){
        ROS_WARN("ros2qnx bind local UDP port %d failed", local_port);
      }else{
        ROS_INFO("ros2qnx local UDP port bound: %d", local_port);
      }
    }

    addr_qnx_.sin_family = AF_INET;
    addr_qnx_.sin_port = htons(remote_port);
    addr_qnx_.sin_addr.s_addr = inet_addr(remote_ip.c_str());
    ROS_INFO("ros2qnx UDP target %s:%d, vel factors x=%.3f y=%.3f yaw=%.3f",
             remote_ip.c_str(), remote_port, vel_x_factor_, vel_y_factor_, vel_yaw_factor_);
  }

  void CmdVelCallback(geometry_msgs::TwistConstPtr msg){
    int nbytes;
    ComplexCMD complexcmd;
    complexcmd.cmd_code = 320;
    complexcmd.cmd_value = 8;
    complexcmd.type = 1;
    complexcmd.data = msg->linear.x * vel_x_factor_;                  ///< linear x velocity
    nbytes = sendto(fd_, &complexcmd, sizeof(complexcmd), 0,
        (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));
    WarnSendError(nbytes, complexcmd.cmd_code);

    complexcmd.cmd_code =325;
    complexcmd.cmd_value = 8;
    complexcmd.type = 1;
    complexcmd.data = msg->linear.y * vel_y_factor_;                 ///< linear y velocity
    nbytes = sendto(fd_, &complexcmd, sizeof(complexcmd), 0,
        (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));
    WarnSendError(nbytes, complexcmd.cmd_code);

    complexcmd.cmd_code = 321;
    complexcmd.cmd_value = 8;
    complexcmd.type = 1;
    complexcmd.data = -msg->angular.z * vel_yaw_factor_;           ///< angular velocity
    nbytes = sendto(fd_, &complexcmd, sizeof(complexcmd), 0,
        (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));
    WarnSendError(nbytes, complexcmd.cmd_code);

    if(isdebug_){
      ROS_INFO_THROTTLE(0.5, "ros2qnx /cmd_vel x=%.3f y=%.3f yaw=%.3f -> qnx x=%.3f y=%.3f yaw=%.3f",
                        msg->linear.x, msg->linear.y, msg->angular.z,
                        msg->linear.x * vel_x_factor_,
                        msg->linear.y * vel_y_factor_,
                        -msg->angular.z * vel_yaw_factor_);
    }
  }

  void KickBallCallback(std_msgs::Int32 msg){
    SimpleCMD cmd;
    cmd.cmd_code = 503;
    cmd.cmd_value = msg.data;
    cmd.type = 0;
    int nbytes = sendto(fd_, &cmd, sizeof(cmd), 0,
        (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));
  }

  void SimpleCMDCallback(message_transformer::SimpleCMD msg){
    SimpleCMD cmd;
    cmd.cmd_code = msg.cmd_code;
    cmd.cmd_value = msg.cmd_value;
    cmd.type = msg.type;
    int nbytes = sendto(fd_, &cmd, sizeof(cmd), 0,
        (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));
  }

  void ComplexCMDCallback(message_transformer::ComplexCMD msg){
    ComplexCMD cmd;
    cmd.cmd_code = msg.cmd_code;
    cmd.cmd_value = msg.cmd_value;
    cmd.type = msg.type;
    cmd.data = msg.data;
    int nbytes = sendto(fd_, &cmd, sizeof(cmd), 0,
        (struct sockaddr *)&addr_qnx_, sizeof(addr_qnx_));
  }

private:
  void WarnSendError(int nbytes, int cmd_code){
    if(nbytes < 0){
      ROS_WARN_THROTTLE(1.0, "sendto failed for cmd_code=%d", cmd_code);
    }
  }

  int fd_=-1;
  struct sockaddr_in addr_qnx_;
  double vel_x_factor_ = 1.0;
  double vel_y_factor_ = 1.0;
  double vel_yaw_factor_ = 1.0;
  bool isdebug_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros2qnx");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS2QNX ros2qnx(pnh);
  ROS_INFO("-----   ros2qnx node up   -----");
  ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 1, &ROS2QNX::CmdVelCallback, &ros2qnx);
  ros::Subscriber vel_sub2 = nh.subscribe("cmd_vel_corrected", 1, &ROS2QNX::CmdVelCallback, &ros2qnx);
  ros::Subscriber kickball_sub = nh.subscribe("kick_ball", 1, &ROS2QNX::KickBallCallback, &ros2qnx);
  ros::Subscriber simplecmd_sub = nh.subscribe("simple_cmd", 1, &ROS2QNX::SimpleCMDCallback, &ros2qnx);
  ros::Subscriber complexcmd_sub = nh.subscribe("complex_cmd", 1, &ROS2QNX::ComplexCMDCallback, &ros2qnx);

  ros::spin();

  return 0;
}
