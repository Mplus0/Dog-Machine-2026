#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <string>

// 定义一个类来封装ROS节点的功能
class OdometryRemapper {
public:
  // 构造函数
  OdometryRemapper() : nh_("~") {
    // 从参数服务器读取订阅的话题名字
    std::string sub_topic_name;
    nh_.param<std::string>("sub_topic", sub_topic_name, "/leg_odom2");
    
    // 从参数服务器读取发布的话题名字
    std::string pub_topic_name;
    nh_.param<std::string>("pub_topic", pub_topic_name, "/odom_add_frame");

    // 从参数服务器读取 frame_id 和 child_frame_id
    nh_.param<std::string>("frame_id", frame_id_, "odom");
    nh_.param<std::string>("child_frame_id", child_frame_id_, "base_link");

    // 订阅话题，当接收到消息时，调用 odomCallback 函数
    odom_sub_ = nh_.subscribe(sub_topic_name, 1, &OdometryRemapper::odomCallback, this);

    // 发布到话题，该话题将包含修正后的里程计消息
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(pub_topic_name, 1);

    ROS_INFO("Odometry Remapper Node Started.");
    ROS_INFO("Subscribing to: %s", sub_topic_name.c_str());
    ROS_INFO("Publishing to: %s", pub_topic_name.c_str());
    ROS_INFO("Using frame_id: %s and child_frame_id: %s", frame_id_.c_str(), child_frame_id_.c_str());
  }

  // 回调函数，用于处理接收到的里程计消息
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 创建一个新的 Odometry 消息，并从接收到的消息中复制数据
    nav_msgs::Odometry filtered_odom = *msg;

    // 修改 frame_id 和 child_frame_id 以符合 ROS 导航栈的惯例
    filtered_odom.header.stamp = ros::Time::now();
    filtered_odom.header.frame_id = frame_id_;
    filtered_odom.child_frame_id = child_frame_id_;

    // 重新发布修改后的消息到新的话题
    odom_pub_.publish(filtered_odom);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Publisher odom_pub_;
  std::string frame_id_;
  std::string child_frame_id_;
};

int main(int argc, char** argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "odom_add__frame_node");

  // 创建 OdometryRemapper 类的实例
  OdometryRemapper remapper;

  // 进入ROS事件循环，等待消息
  ros::spin();

  return 0;
}
