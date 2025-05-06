#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <waypoint_manager/Waypoint.h>
#include <waypoint_manager/GetWaypoints.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class WaypointVisualizer {
public:
    WaypointVisualizer(ros::NodeHandle& nh) : nh_(nh) {
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("waypoint_markers", 1);
        get_waypoints_client_ = nh_.serviceClient<waypoint_manager::GetWaypoints>("get_waypoints");
        
        // Create timer for periodic updates
        timer_ = nh_.createTimer(ros::Duration(1.0), &WaypointVisualizer::updateMarkers, this);
    }

private:
    void updateMarkers(const ros::TimerEvent& event) {
        waypoint_manager::GetWaypoints srv;
        if (!get_waypoints_client_.call(srv)) {
            ROS_WARN("Failed to get waypoints");
            return;
        }

        visualization_msgs::MarkerArray marker_array;
        int id = 0;

        for (const auto& waypoint : srv.response.waypoints) {
            // Create arrow marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "waypoints";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = waypoint.pose;
            marker.scale.x = 0.5;  // Arrow length
            marker.scale.y = 0.1;  // Arrow width
            marker.scale.z = 0.1;  // Arrow height

            // Set color based on creation type
            if (waypoint.creation_type == waypoint_manager::Waypoint::CREATION_TYPE_MANUAL) {
                marker.color.r = 1.0;  // Red for manual waypoints
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            } else {
                marker.color.r = 0.0;
                marker.color.g = 1.0;  // Green for automatic waypoints
                marker.color.b = 0.0;
            }
            marker.color.a = 1.0;

            // Add text marker for waypoint name
            visualization_msgs::Marker text_marker;
            text_marker.header = marker.header;
            text_marker.ns = "waypoint_names";
            text_marker.id = id++;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.pose = waypoint.pose;
            text_marker.pose.position.z += 0.5;  // Place text above the arrow
            text_marker.scale.z = 0.2;  // Text size
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            text_marker.text = waypoint.name;

            marker_array.markers.push_back(marker);
            marker_array.markers.push_back(text_marker);
        }

        marker_pub_.publish(marker_array);
    }

    ros::NodeHandle& nh_;
    ros::Publisher marker_pub_;
    ros::ServiceClient get_waypoints_client_;
    ros::Timer timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_visualizer");
    ros::NodeHandle nh;
    WaypointVisualizer visualizer(nh);
    ros::spin();
    return 0;
} 