#include <ros/ros.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h> // Make sure this is included
#include <fstream>
#include <string>
#include <iomanip>

// Global variables (unchanged)
std::string yaml_file_path;
std::string current_goal_name = "";

// The corrected goalNameCallback function, moved to be defined before main()
void goalNameCallback(const std_msgs::String::ConstPtr& msg) {
    current_goal_name = msg->data;
    ROS_INFO("Goal name '%s' received. Waiting for navigation to succeed...", current_goal_name.c_str());
}

// statusCallback function (unchanged)
void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
    // ... (rest of the function is the same)
    if (msg->status_list.empty()) {
        return;
    }

    const actionlib_msgs::GoalStatus& last_status = msg->status_list.back();

    if (last_status.status == actionlib_msgs::GoalStatus::SUCCEEDED) {
        ROS_INFO("Navigation to a goal has succeeded!");

        if (current_goal_name.empty()) {
            ROS_WARN("A navigation goal succeeded, but no name was provided. Cannot save pose.");
            return;
        }

        tf::TransformListener listener;
        tf::StampedTransform transform;

        try {
            listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
            listener.lookupTransform("map", "base_link", ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("Failed to get transform from map to base_link: %s", ex.what());
            return;
        }

        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double z = transform.getOrigin().z();
        tf::Quaternion q = transform.getRotation();
        double qx = q.x();
        double qy = q.y();
        double qz = q.z();
        double qw = q.w();

        ROS_INFO("Final pose in 'map' frame:");
        ROS_INFO("  - Position: x=%.3f, y=%.3f, z=%.3f", x, y, z);
        ROS_INFO("  - Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", qx, qy, qz, qw);

        std::ofstream outfile(yaml_file_path, std::ios_base::app);
        if (!outfile.is_open()) {
            ROS_ERROR("Could not open YAML file for writing: %s", yaml_file_path.c_str());
            return;
        }

        outfile << "  " << current_goal_name << ":" << std::endl;
        outfile << "    position:" << std::endl;
        outfile << "      x: " << std::fixed << std::setprecision(3) << x << std::endl;
        outfile << "      y: " << std::fixed << std::setprecision(3) << y << std::endl;
        outfile << "      z: " << std::fixed << std::setprecision(3) << z << std::endl;
        outfile << "    orientation:" << std::endl;
        outfile << "      x: " << std::fixed << std::setprecision(3) << qx << std::endl;
        outfile << "      y: " << std::fixed << std::setprecision(3) << qy << std::endl;
        outfile << "      z: " << std::fixed << std::setprecision(3) << qz << std::endl;
        outfile << "      w: " << std::fixed << std::setprecision(3) << qw << std::endl;
        outfile << std::endl;

        outfile.close();
        ROS_INFO("Successfully saved goal '%s' to %s", current_goal_name.c_str(), yaml_file_path.c_str());
        
        current_goal_name = "";
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "save_success_pose_node");
    ros::NodeHandle nh("~");

    if (!nh.getParam("yaml_file_path", yaml_file_path)) {
        ROS_ERROR("Failed to get 'yaml_file_path' parameter. Please set it in the launch file.");
        return 1;
    }

    // Now goalNameCallback is in scope and can be used
    ros::Subscriber name_sub = nh.subscribe("/SaveGoalWithName", 1, goalNameCallback);
    
    // The status_sub can also be created here
    ros::Subscriber status_sub = nh.subscribe("/move_base/status", 1, statusCallback);


    ROS_INFO("Save Success Pose Node is ready. Waiting for successful navigation...");

    ros::spin();
    return 0;
}