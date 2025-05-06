#include "waypoint_manager/waypoint_manager.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ros/package.h>
#include <cmath>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <interactive_markers/interactive_marker_server.h>


WaypointManager::WaypointManager(ros::NodeHandle& nh) : 
    nh_(nh),
    tf_listener_(tf_buffer_),
    move_base_client_("move_base", true),
    next_waypoint_id_(1),
    is_first_pose_(true)
{
    // Load parameters
    nh_.param("position_threshold", position_threshold_, 0.5);
    nh_.param("distance_threshold", distance_threshold_, 2.0);
    nh_.param("angle_threshold", angle_threshold_, M_PI/4.0); // 45 degrees

    // publishers and subscribers
    odom_sub_ = nh_.subscribe("odom", 1, &WaypointManager::odomCallback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("waypoint_markers", 1);
    publish_point_sub_ = nh_.subscribe("clicked_point", 1, &WaypointManager::publishPointCallback, this);
    
    // services
    create_waypoint_srv_ = nh_.advertiseService("create_waypoint", &WaypointManager::createWaypointCallback, this);
    get_waypoints_srv_ = nh_.advertiseService("get_waypoints", &WaypointManager::getWaypointsCallback, this);
    navigate_to_waypoint_srv_ = nh_.advertiseService("navigate_to_waypoint", &WaypointManager::navigateToWaypointCallback, this);
    
    // clients
    navigate_to_waypoint_client_ = nh_.serviceClient<waypoint_manager::NavigateToWaypoint>("navigate_to_waypoint");
    create_waypoint_client_ = nh_.serviceClient<waypoint_manager::CreateWaypoint>("create_waypoint");
    
    // interactive marker server
    marker_server_ = new interactive_markers::InteractiveMarkerServer("waypoint_markers");


    loadWaypoints();

    // Wait for move_base action server
    ROS_INFO("Waiting for move_base action server...");
    move_base_client_.waitForServer();
    ROS_INFO("move_base action server connected!");
    
    
    ROS_INFO("WaypointManager initialized!");
}
//tested
WaypointManager::~WaypointManager() {
    saveWaypoints();
    // raushantodo:  check the delete function later 
    delete marker_server_;
}
//tested



void WaypointManager::publishPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    ROS_INFO("\n\n--------------------------------\n\n");
    ROS_INFO("publishPointCallback: %f, %f, %f", msg->point.x, msg->point.y, msg->point.z);
    
    waypoint_manager::CreateWaypoint create_waypoint_srv;
    create_waypoint_srv.request.name = "Manual Waypoint " + std::to_string(next_waypoint_id_);
    create_waypoint_client_.call(create_waypoint_srv);
    ROS_INFO("create_waypoint_srv: %s", create_waypoint_srv.response.message.c_str());
    ROS_INFO("--------------------------------");
}

void WaypointManager::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (is_first_pose_) {
        last_pose_ = msg->pose.pose;
        is_first_pose_ = false;
        return;
    }

    geometry_msgs::Pose current_pose = msg->pose.pose;
    // ROS_INFO("current_pose: %f, %f, %f", current_pose.position.x, current_pose.position.y, current_pose.position.z);
    double distance = calculateDistance(last_pose_, current_pose);
    double angle = calculateAngle(last_pose_, current_pose);
    // ROS_INFO("distance: %f, angle: %f", distance, angle*180/M_PI);
    if (distance >= distance_threshold_ || angle >= angle_threshold_) {
        if (!isNearExistingWaypoint(current_pose)) {
            createWaypoint(current_pose, waypoint_manager::Waypoint::CREATION_TYPE_AUTO);
        }
        last_pose_ = current_pose;
    }
}

bool WaypointManager::createWaypointCallback(waypoint_manager::CreateWaypoint::Request& req,
                                           waypoint_manager::CreateWaypoint::Response& res) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";
    pose_stamped.header.stamp = ros::Time::now();

    try {
        // Get the robot's current pose in the map frame
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
        pose_stamped.pose.position.x = transform.transform.translation.x;
        pose_stamped.pose.position.y = transform.transform.translation.y;
        pose_stamped.pose.position.z = transform.transform.translation.z;
        pose_stamped.pose.orientation = transform.transform.rotation;
    } catch (tf2::TransformException& ex) {
        res.success = false;
        res.message = "Failed to get robot pose: " + std::string(ex.what());
        return false;
    }

    if (isNearExistingWaypoint(pose_stamped.pose)) {
        res.success = false;
        res.message = "Too close to existing waypoint";
        return false;
    }

    createWaypoint(pose_stamped.pose, waypoint_manager::Waypoint::CREATION_TYPE_MANUAL, req.name);
    res.success = true;
    res.message = "Waypoint created successfully";
    return true;
}

bool WaypointManager::getWaypointsCallback(waypoint_manager::GetWaypoints::Request& req,
                                         waypoint_manager::GetWaypoints::Response& res) {
    res.waypoints.clear();
    for (const auto& pair : waypoints_) {
        res.waypoints.push_back(pair.second);
    }
    return true;
}


bool WaypointManager::navigateToWaypointCallback(waypoint_manager::NavigateToWaypoint::Request& req,
                                               waypoint_manager::NavigateToWaypoint::Response& res) {
    
    ROS_INFO("navigateToWaypointCallback: %d", req.waypoint_id);
    if (waypoints_.find(req.waypoint_id) == waypoints_.end()) {
        ROS_ERROR("Waypoint not found: %d", req.waypoint_id);
        res.success = false;
        res.message = "Waypoint not found";
        return false;
    }

    // Find current waypoint
    geometry_msgs::PoseStamped current_pose;
    current_pose.header.frame_id = "map";
    current_pose.header.stamp = ros::Time::now();

    try {
        // Get the robot's current pose in the map frame
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            "map", "base_link", ros::Time(0));
        current_pose.pose.position.x = transform.transform.translation.x;
        current_pose.pose.position.y = transform.transform.translation.y;
        current_pose.pose.position.z = transform.transform.translation.z;
        current_pose.pose.orientation = transform.transform.rotation;
    } catch (tf2::TransformException& ex) {
        res.success = false;
        res.message = "Failed to get robot pose: " + std::string(ex.what());
        return false;
    }

    // Find closest waypoint to current pose
    uint32_t start_id = 0;
    double min_distance = std::numeric_limits<double>::max();
    for (const auto& pair : waypoints_) {
        double distance = calculateDistance(current_pose.pose, pair.second.pose);
        if (distance < min_distance) {
            min_distance = distance;
            start_id = pair.first;
        }
    }

    // Find path to goal
    std::vector<uint32_t> path = findPath(start_id, req.waypoint_id);
    //check if path is empty
    if (path.empty()) {
        res.success = false;
        res.message = "No path found to waypoint";
        return false;
    }

    // Navigate through waypoints
    for (size_t i = 1; i < path.size(); ++i) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = waypoints_[path[i]].pose;

        move_base_client_.sendGoal(goal);
        move_base_client_.waitForResult();

        if (move_base_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
            res.success = false;
            res.message = "Failed to reach waypoint " + std::to_string(path[i]);
            return false;
        }
    }

    res.success = true;
    res.message = "Successfully reached waypoint " + std::to_string(req.waypoint_id);
    return true;
}

//tested
void WaypointManager::createWaypoint(const geometry_msgs::Pose& pose, uint8_t creation_type, const std::string& name) {
    waypoint_manager::Waypoint waypoint;
    waypoint.id = next_waypoint_id_++;
    waypoint.pose = pose;
    waypoint.creation_type = creation_type;
    waypoint.name = name.empty() ? "Waypoint " + std::to_string(waypoint.id) : name;
    ROS_INFO("waypoint: %s", waypoint.name.c_str());

    waypoints_[waypoint.id] = waypoint;
    updateVisualization();
    saveWaypoints();
}

//tested but marker is not visible , data is not comming in topic 
void WaypointManager::updateVisualization() {
    // Clear existing markers
    // raushantodo:  check the clear function later 
    // marker_server_->clear();  // Use -> since marker_server_ is a pointer to InteractiveMarkerServer

    for (const auto& pair : waypoints_) {
        const waypoint_manager::Waypoint& waypoint = pair.second;
        // Create interactive marker
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = "map";
        int_marker.header.stamp = ros::Time::now();
        //check1
        int_marker.name = "waypoint_" + std::to_string(waypoint.id);
        int_marker.description = waypoint.name;
        int_marker.pose = waypoint.pose;

        visualization_msgs::InteractiveMarkerControl button_control;
        button_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
        button_control.name = "button_" + std::to_string(waypoint.id);
        // ROS_INFO("button_control.name: %s", button_control.name.c_str());
        button_control.always_visible = true;
        int_marker.controls.push_back(button_control);

        // Create marker
        visualization_msgs::Marker box_marker;
        box_marker.type = visualization_msgs::Marker::CUBE;
        box_marker.scale.x = 0.5;
        box_marker.scale.y = 0.5;
        box_marker.scale.z = 0.5;
        // different color for manual and auto waypoints
        if (waypoint.creation_type == waypoint_manager::Waypoint::CREATION_TYPE_MANUAL) {
            box_marker.color.r = 1.0;
            box_marker.color.g = 0.0;
        } else {
            box_marker.color.r = 0.0;
            box_marker.color.g = 1.0;
        }
        box_marker.color.b = 0.0;
        box_marker.color.a = 1.0;

        button_control.markers.push_back(box_marker);
        // int_marker.controls.clear();
        int_marker.controls.push_back(button_control);

        marker_server_->insert(int_marker, boost::bind(&WaypointManager::processFeedback, this, _1));
    }
        marker_server_->applyChanges();

}
//tested
void WaypointManager::saveWaypoints() {
    YAML::Node node;
    for (const auto& pair : waypoints_) {
        const waypoint_manager::Waypoint& waypoint = pair.second;
        YAML::Node waypoint_node;
        waypoint_node["id"] = waypoint.id;
        waypoint_node["name"] = waypoint.name;
        waypoint_node["creation_type"] = waypoint.creation_type;
        waypoint_node["pose"]["position"]["x"] = waypoint.pose.position.x;
        waypoint_node["pose"]["position"]["y"] = waypoint.pose.position.y;
        waypoint_node["pose"]["position"]["z"] = waypoint.pose.position.z;
        waypoint_node["pose"]["orientation"]["x"] = waypoint.pose.orientation.x;
        waypoint_node["pose"]["orientation"]["y"] = waypoint.pose.orientation.y;
        waypoint_node["pose"]["orientation"]["z"] = waypoint.pose.orientation.z;
        waypoint_node["pose"]["orientation"]["w"] = waypoint.pose.orientation.w;
        node["waypoints"].push_back(waypoint_node);
    }

    std::string package_path = ros::package::getPath("waypoint_manager");
    std::string file_path = package_path + "/config/waypoints.yaml";
    
    try {
        std::ofstream fout(file_path);
        if (!fout.is_open()) {
            ROS_ERROR("Failed to open waypoints file for writing: %s", file_path.c_str());
            return;
        }
        fout << node;
        fout.close();
        // ROS_INFO("Waypoints saved to %s", file_path.c_str());
        /*for (const auto& pair : waypoints_) {
            ROS_INFO("--------------------------------");
            ROS_INFO("waypoint: %s", pair.second.name.c_str());
            ROS_INFO("pose: %f, %f, %f", pair.second.pose.position.x, pair.second.pose.position.y, pair.second.pose.position.z);
            ROS_INFO("--------------------------------");
        }*/
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to save waypoints: %s", e.what());
    }
}

//tested
void WaypointManager::loadWaypoints() {
    std::string package_path = ros::package::getPath("waypoint_manager");
    std::string file_path = package_path + "/config/waypoints.yaml";
    
    try {
        if (!std::ifstream(file_path)) {
            ROS_WARN("Waypoints file does not exist: %s", file_path.c_str());
            return;
        }
        
        YAML::Node node = YAML::LoadFile(file_path);
        if (node["waypoints"]) {
            for (const auto& waypoint_node : node["waypoints"]) {
                waypoint_manager::Waypoint waypoint;
                waypoint.id = waypoint_node["id"].as<uint32_t>();
                waypoint.name = waypoint_node["name"].as<std::string>();
                waypoint.creation_type = waypoint_node["creation_type"].as<uint8_t>();
                waypoint.pose.position.x = waypoint_node["pose"]["position"]["x"].as<double>();
                waypoint.pose.position.y = waypoint_node["pose"]["position"]["y"].as<double>();
                waypoint.pose.position.z = waypoint_node["pose"]["position"]["z"].as<double>();
                waypoint.pose.orientation.x = waypoint_node["pose"]["orientation"]["x"].as<double>();
                waypoint.pose.orientation.y = waypoint_node["pose"]["orientation"]["y"].as<double>();
                waypoint.pose.orientation.z = waypoint_node["pose"]["orientation"]["z"].as<double>();
                waypoint.pose.orientation.w = waypoint_node["pose"]["orientation"]["w"].as<double>();

                waypoints_[waypoint.id] = waypoint;
                if (waypoint.id >= next_waypoint_id_) {
                    next_waypoint_id_ = waypoint.id + 1;
                }
            }
            ROS_INFO("Loaded %zu waypoints from %s", waypoints_.size(), file_path.c_str());
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to load waypoints: %s", e.what());
    }
}

bool WaypointManager::isNearExistingWaypoint(const geometry_msgs::Pose& pose) {
    for (const auto& pair : waypoints_) {
        if (calculateDistance(pose, pair.second.pose) < position_threshold_) {
            return true;
        }
    }
    return false;
}

double WaypointManager::calculateDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    return std::sqrt(dx*dx + dy*dy);
}

double WaypointManager::calculateAngle(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
    tf2::Quaternion q1(pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w);
    tf2::Quaternion q2(pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w);
    
    tf2::Matrix3x3 m1(q1);
    tf2::Matrix3x3 m2(q2);
    
    double roll1, pitch1, yaw1;
    double roll2, pitch2, yaw2;
    
    m1.getRPY(roll1, pitch1, yaw1);
    m2.getRPY(roll2, pitch2, yaw2);
    
    return std::abs(yaw2 - yaw1);
}

std::vector<uint32_t> WaypointManager::findPath(uint32_t start_id, uint32_t goal_id) {
    // Simple implementation using A* algorithm
    std::map<uint32_t, double> g_score;
    std::map<uint32_t, double> f_score;
    std::map<uint32_t, uint32_t> came_from;
    std::set<uint32_t> open_set;
    std::set<uint32_t> closed_set;

    g_score[start_id] = 0;
    f_score[start_id] = calculateDistance(waypoints_[start_id].pose, waypoints_[goal_id].pose);
    open_set.insert(start_id);

    while (!open_set.empty()) {
        // Find node with lowest f_score
        uint32_t current = *open_set.begin();
        double lowest_f = f_score[current];
        for (uint32_t id : open_set) {
            if (f_score[id] < lowest_f) {
                current = id;
                lowest_f = f_score[id];
            }
        }

        if (current == goal_id) {
            // Reconstruct path
            std::vector<uint32_t> path;
            while (current != start_id) {
                path.push_back(current);
                current = came_from[current];
            }
            path.push_back(start_id);
            std::reverse(path.begin(), path.end());
            return path;
        }

        open_set.erase(current);
        closed_set.insert(current);

        // Check all waypoints as potential neighbors
        for (const auto& pair : waypoints_) {
            uint32_t neighbor = pair.first;
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }

            double tentative_g_score = g_score[current] + 
                calculateDistance(waypoints_[current].pose, waypoints_[neighbor].pose);

            if (open_set.find(neighbor) == open_set.end()) {
                open_set.insert(neighbor);
            } else if (tentative_g_score >= g_score[neighbor]) {
                continue;
            }

            came_from[neighbor] = current;
            g_score[neighbor] = tentative_g_score;
            f_score[neighbor] = g_score[neighbor] + 
                calculateDistance(waypoints_[neighbor].pose, waypoints_[goal_id].pose);
        }
    }

    return std::vector<uint32_t>();
}

std::vector<uint32_t> WaypointManager::findPathDijkstra(uint32_t start_id, uint32_t goal_id) {
    std::map<uint32_t, double> dist;
    std::map<uint32_t, uint32_t> prev;
    std::set<uint32_t> visited;
    using Pair = std::pair<double, uint32_t>;
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> pq;

    for (const auto& wp : waypoints_) {
        dist[wp.first] = std::numeric_limits<double>::infinity();
    }
    dist[start_id] = 0.0;
    pq.push({0.0, start_id});

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (visited.count(u)) continue;
        visited.insert(u);

        if (u == goal_id) break;

        for (const auto& v : waypoints_) {
            if (u == v.first) continue;
            double cost = calculateDistance(waypoints_[u].pose, waypoints_[v.first].pose);
            if (dist[u] + cost < dist[v.first]) {
                dist[v.first] = dist[u] + cost;
                prev[v.first] = u;
                pq.push({dist[v.first], v.first});
            }
        }
    }

    // Reconstruct path
    std::vector<uint32_t> path;
    if (prev.find(goal_id) == prev.end()) return path; // No path found
    for (uint32_t at = goal_id; at != start_id; at = prev[at]) {
        path.push_back(at);
    }
    path.push_back(start_id);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<uint32_t> WaypointManager::findPathBFS(uint32_t start_id, uint32_t goal_id) {
    std::map<uint32_t, uint32_t> prev;
    std::set<uint32_t> visited;
    std::queue<uint32_t> q;

    q.push(start_id);
    visited.insert(start_id);

    while (!q.empty()) {
        uint32_t u = q.front(); q.pop();
        if (u == goal_id) break;

        for (const auto& v : waypoints_) {
            if (u == v.first) continue;
            if (visited.count(v.first)) continue;
            // Optionally, only connect "nearby" waypoints by distance threshold
            // double dist = calculateDistance(waypoints_[u].pose, waypoints_[v.first].pose);
            // if (dist > SOME_THRESHOLD) continue;

            prev[v.first] = u;
            visited.insert(v.first);
            q.push(v.first);
        }
    }

    // Reconstruct path
    std::vector<uint32_t> path;
    if (prev.find(goal_id) == prev.end()) return path; // No path found
    for (uint32_t at = goal_id; at != start_id; at = prev[at]) {
        path.push_back(at);
    }
    path.push_back(start_id);
    std::reverse(path.begin(), path.end());
    return path;
}


void WaypointManager::processFeedback1(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    ROS_INFO("---------------processFeedback1  start-----------------");   
 if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK) {
        ROS_INFO("Marker was clicked!");
    }
    ROS_INFO("---------------processFeedback1  end-----------------");   
}

void WaypointManager::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
    // ROS_INFO("---------------processFeedback  start-----------------");   
    // Log the feedback event type and marker name for debugging
    // ROS_INFO("Received feedback for marker: %s", feedback->marker_name.c_str());
    // ROS_INFO("Event type: %d", feedback->event_type);
    
    // Print more detailed feedback information
    // switch(feedback->event_type) {
    //     case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    //         ROS_INFO("Button clicked on waypoint marker!");
    //         break;
    //     case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
    //         ROS_INFO("Mouse down on waypoint marker");
    //         break;
    //     case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
    //         ROS_INFO("Mouse up on waypoint marker");
    //         break;
    //     default:
    //         ROS_INFO("Other interaction with waypoint marker");
    //         break;
    // }
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK) {
        ROS_INFO("feedback->event_type: %d \t button clicked on waypoint marker", feedback->event_type);
        // Extract waypoint ID from marker name
        std::string id_str = feedback->marker_name.substr(9); // Remove "waypoint_" prefix
        uint32_t waypoint_id = std::stoul(id_str);
        ROS_INFO("detected waypoint_id: %d", waypoint_id);
        // Create navigation request

        
        waypoint_manager::NavigateToWaypoint srv;
        srv.request.waypoint_id = waypoint_id; //waypoint_id;
        navigate_to_waypoint_client_.waitForExistence();
        ROS_INFO("navigate_to_waypoint_client_  calling service");
        navigate_to_waypoint_client_.call(srv);
        ROS_INFO("Navigation result: %s", srv.response.message.c_str());
        // if (navigate_to_waypoint_client_.call(srv)) {
        //     ROS_INFO("Navigation result: %s", srv.response.message.c_str());
        // } else {
        //     ROS_ERROR("Failed to call navigate_to_waypoint service");
        // }
    }
    // ROS_INFO("---------------processFeedback  end-----------------");   
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_manager");
    ros::NodeHandle nh;
    WaypointManager waypoint_manager(nh);
    ros::AsyncSpinner spinner(3); // client is blocking the thread
    spinner.start();
    ros::waitForShutdown();
    return 0;
} 