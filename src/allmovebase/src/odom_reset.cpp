#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <string>

// 定义一个类来封装ROS节点的功能
class OdometryResetter {
public:
  // 构造函数
  OdometryResetter() : 
    nh_("~"), 
    is_initialized_(false),
    sub_topic_name_("/odom_add_frame"),
    pub_topic_name_("/odom_reset"),
    frame_id_("odom"),
    child_frame_id_("base_link") 
  {
    // 订阅话题，当接收到消息时，调用 odomCallback 函数
    odom_sub_ = nh_.subscribe(sub_topic_name_, 1, &OdometryResetter::odomCallback, this);

    // 发布到话题，该话题将包含重置后的里程计消息
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(pub_topic_name_, 1);

    ROS_INFO("Odometry Resetter Node Started.");
    ROS_INFO("Subscribing to: %s", sub_topic_name_.c_str());
    ROS_INFO("Publishing to: %s", pub_topic_name_.c_str());
    ROS_INFO("Using frame_id: %s and child_frame_id: %s", frame_id_.c_str(), child_frame_id_.c_str());
  }

  // 回调函数，用于处理接收到的里程计消息
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 第一次收到消息时，初始化
    if (!is_initialized_) {
      // 记录初始位姿
      initial_pose_ = msg->pose.pose;
      is_initialized_ = true;
      ROS_INFO("Initial odometry pose recorded. Resetting point set.");
    }

    // 创建一个新的 Odometry 消息
    nav_msgs::Odometry reset_odom;
    reset_odom.header = msg->header;
    reset_odom.header.frame_id = frame_id_; // 修改 frame_id
    reset_odom.child_frame_id = child_frame_id_; // 修改 child_frame_id

    // 计算相对于初始位姿的新位姿
    // 获取当前位姿和初始位姿的tf变换
    tf::Pose current_pose, initial_pose;
    tf::poseMsgToTF(msg->pose.pose, current_pose);
    tf::poseMsgToTF(initial_pose_, initial_pose);

    // 计算相对变换：T_relative = T_initial.inverse() * T_current
    tf::Pose relative_pose = initial_pose.inverse() * current_pose;

    // 将相对变换转换回 geometry_msgs::Pose
    tf::poseTFToMsg(relative_pose, reset_odom.pose.pose);

    // 复制其他信息，例如协方差和扭曲
    reset_odom.pose.covariance = msg->pose.covariance;
    reset_odom.twist = msg->twist;

    // 发布重置后的里程计消息
    odom_pub_.publish(reset_odom);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Publisher odom_pub_;
  std::string sub_topic_name_;
  std::string pub_topic_name_;
  std::string frame_id_;
  std::string child_frame_id_;
  geometry_msgs::Pose initial_pose_;
  bool is_initialized_;
};

int main(int argc, char** argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "odom_reset_node");

  // 创建 OdometryResetter 类的实例
  OdometryResetter resetter;

  // 进入ROS事件循环
  ros::spin();

  return 0;
}