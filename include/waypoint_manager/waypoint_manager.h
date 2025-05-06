#ifndef WAYPOINT_MANAGER_H
#define WAYPOINT_MANAGER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <waypoint_manager/Waypoint.h>
#include <waypoint_manager/CreateWaypoint.h>
#include <waypoint_manager/GetWaypoints.h>
#include <waypoint_manager/NavigateToWaypoint.h>
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>

class WaypointManager {
public:
    WaypointManager(ros::NodeHandle& nh);
    ~WaypointManager();

private:
    // ROS related members
    ros::NodeHandle& nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber publish_point_sub_;
    ros::Publisher marker_pub_;
    ros::ServiceServer create_waypoint_srv_;
    ros::ServiceServer get_waypoints_srv_;
    ros::ServiceServer navigate_to_waypoint_srv_;
    ros::ServiceClient navigate_to_waypoint_client_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    interactive_markers::InteractiveMarkerServer* marker_server_;

    // Action client for move_base
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;

    // Waypoint management
    std::map<uint32_t, waypoint_manager::Waypoint> waypoints_;
    uint32_t next_waypoint_id_;
    double position_threshold_;
    double distance_threshold_;
    double angle_threshold_;
    geometry_msgs::Pose last_pose_;
    bool is_first_pose_;

    // Callback functions
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    bool createWaypointCallback(waypoint_manager::CreateWaypoint::Request& req,
                              waypoint_manager::CreateWaypoint::Response& res);
    bool getWaypointsCallback(waypoint_manager::GetWaypoints::Request& req,
                            waypoint_manager::GetWaypoints::Response& res);
    bool navigateToWaypointCallback(waypoint_manager::NavigateToWaypoint::Request& req,
                                  waypoint_manager::NavigateToWaypoint::Response& res);
    void processFeedback1(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    // Helper functions
    void createWaypoint(const geometry_msgs::Pose& pose, uint8_t creation_type, const std::string& name = "");
    void updateVisualization();
    void saveWaypoints();
    void loadWaypoints();
    bool isNearExistingWaypoint(const geometry_msgs::Pose& pose);
    double calculateDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2);
    double calculateAngle(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2);
    std::vector<uint32_t> findPath(uint32_t start_id, uint32_t goal_id);
};

#endif // WAYPOINT_MANAGER_H 