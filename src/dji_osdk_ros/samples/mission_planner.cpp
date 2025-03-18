#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <dji_osdk_ros/MissionAction.h>
#include <dji_osdk_ros/common_type.h>
#include <string>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPoint.h>

#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/SetAvoidEnable.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>
#include <dji_osdk_ros/GetAvoidEnable.h>
#include <Eigen/Dense>
#include <dji_osdk_ros/MoveToWaypointAction.h>
#include <std_msgs/UInt8.h>
#include <functional>

namespace osdk = dji_osdk_ros;

namespace impulse_control
{

enum FlightStatus
{
    ON_GROUND = 0,
    TAKING_OFF = 1,
    FLYING = 2,
    LANDING = 3
};

// Define the Action Server class
class MissionPlannerActionServer
{
public:
    MissionPlannerActionServer(std::string name) :
        as_(nh_, name, false),
        action_name_(name),
        ac_("waypoint_control", true)
    {
        task_control_client_ = nh_.serviceClient<osdk::FlightTaskControl>("/flight_task_control");;
        enable_horizon_avoid_client_ = nh_.serviceClient<osdk::SetAvoidEnable>("/set_horizon_avoid_enable");
        enable_upward_avoid_client_ = nh_.serviceClient<osdk::SetAvoidEnable>("/set_upwards_avoid_enable");
        get_avoid_enable_client_ = nh_.serviceClient<osdk::GetAvoidEnable>("get_avoid_enable_status");
        obtain_ctrl_authority_client_ = nh_.serviceClient<osdk::ObtainControlAuthority>("obtain_release_control_authority");

        as_.registerGoalCallback(std::bind(&MissionPlannerActionServer::goalCallback, this));
        as_.registerPreemptCallback(std::bind(&MissionPlannerActionServer::preemptCallback, this));
        as_.start();
        ROS_INFO("Mission Planner Action Server started");
    }

private:
    // Execute mission (takeoff -> waypoints -> landing)
    void goalCallback()
    {
        const auto goal = as_.acceptNewGoal();
        // validate the mission
        if (!validMission(goal))
        {
            std::string err_msg = "The specified mission is not valid";
            ROS_ERROR("%s", err_msg.c_str());
            result_.success = false;
            result_.message = err_msg;
            as_.setAborted(result_, err_msg);
            return;
        }
        ROS_ERROR("Mission is valid; proceeding");

        // obtain the control authority
        osdk::ObtainControlAuthority obtain_ctrl_authority;
  
        obtain_ctrl_authority.request.enable_obtain = true;
        obtain_ctrl_authority_client_.call(obtain_ctrl_authority);

        if (obtain_ctrl_authority.response.result == false)
        {
            std::string err_msg = "Failed to obtain autonomous control";
            ROS_ERROR("%s", err_msg.c_str());
            result_.success = false;
            result_.message = err_msg;
            as_.setAborted(result_, err_msg);
            return;
        }

        ROS_ERROR("Obtained control authority, proceeding");

        // check the flight status
        const auto flight_status_msg = ros::topic::waitForMessage<std_msgs::UInt8>("dji_osdk_ros/flight_status", nh_, ros::Duration(TOPIC_TIMEOUT_S));

        if (!flight_status_msg)
        {
            // message did not get recieved - unaware of flight status. Fail out
            std::string err_msg = "Failed to get flight status";
            ROS_ERROR("%s", err_msg.c_str());
            result_.success = false;
            result_.message = err_msg;
            as_.setAborted(result_, err_msg);
            return;
        }

        const auto flight_msg = static_cast<FlightStatus>(flight_status_msg->data);

        if (flight_msg == ON_GROUND)
        {
            osdk::FlightTaskControl control_task;
            control_task.request.task = osdk::FlightTaskControl::Request::TASK_TAKEOFF;
            ROS_ERROR("Takeoff request sending ...");
            task_control_client_.call(control_task);

            if (control_task.response.result == false)
            {
                std::string err_msg = "Takeoff failed!";
                ROS_ERROR("%s", err_msg.c_str());
                result_.success = false;
                as_.setAborted(result_, err_msg);
                return;
            }
            ROS_ERROR("Takeoff successful, proceed to mission planner");
        }
        else if (flight_msg == FLYING)
        {
            ROS_ERROR("Drone is already flying, proceed");
        }
        else
        {
            // message did not get recieved - unaware of flight status. Fail out
            std::string err_msg = "Unexpected drone status: " + std::to_string(flight_msg);
            ROS_ERROR("%s", err_msg.c_str());
            result_.success = false;
            result_.message = err_msg;
            as_.setAborted(result_, err_msg);
            return;
        }

        ROS_ERROR("Waiting for action server to start");
        ac_.waitForServer();
        ROS_ERROR("Action server started, sending waypoints.");

        waypoints_ = createWaypoints(goal);
        // reset the waypoint index
        waypoint_idx_ = 0;
        navigateToNextWaypoint();
    }

    void preemptCallback()
    {
        ROS_ERROR("Mission preempted");
        as_.setPreempted();
        ac_.cancelAllGoals();
        return;
    }

    std::vector<geometry_msgs::Point> createWaypoints(const osdk::MissionGoalConstPtr& goal)
    {
        // create waypoints in NEU (north east up) as relative goals (to the current location)
        // first move up to flying altitude
        geometry_msgs::Point pt_1, pt_2, pt_3;
        pt_1.x = 0.0;
        pt_1.y = 0.0;
        pt_1.z = FLYING_ALTITUDE_M;

        // then get the NED error and move the drone to (N, E, flight altitude)
        pt_2.x = static_cast<float>(goal_ned_error_.x());
        pt_2.y = static_cast<float>(goal_ned_error_.y());
        pt_2.z = 0.0;

        // drop the drone to (N, E, -D) = (N, E, U) relative position
        pt_3.x = 0.0;
        pt_3.y = 0.0;
        const auto desired_delta_z = -goal_ned_error_.z();
        // desired - current height will give the correct delta z to move the drone down by
        // TODO: Determine if the altitude should be user determined... or just hardset to some value such as (1 - FLYING_ALTITUDE_M)
        pt_3.z = desired_delta_z - FLYING_ALTITUDE_M;

        std::vector<geometry_msgs::Point> waypoints { pt_1, pt_2, pt_3 };
        return waypoints;
    }

    geographic_msgs::GeoPoint navSatFixtoGeoPoint(const sensor_msgs::NavSatFix& gps_pos)
    {
        // TODO: move to helper file later
        geographic_msgs::GeoPoint ret;
        ret.latitude = gps_pos.latitude;
        ret.longitude = gps_pos.longitude;
        ret.altitude = gps_pos.altitude;
        return ret;
    }

    Eigen::Vector3d getNEDError(const geographic_msgs::GeoPoint& current_position, const geographic_msgs::GeoPoint& target_position)
    {
        // TODO: move to helper file later
        // Convert GPS (WGS84) to UTM
        geodesy::UTMPoint ref_utm, target_utm;
        geodesy::fromMsg(current_position, ref_utm);
        geodesy::fromMsg(target_position, target_utm);

        // Calculate NED offsets
        Eigen::Vector3d ned_error { target_utm.northing - ref_utm.northing, target_utm.easting - ref_utm.easting, -(target_position.altitude - current_position.altitude) };
        // ROS_DEBUG_STREAM("The NED Error is N: " << ned_error.x() << ", E: " << ned_error.y() << " D: " << ned_error.z());
        return ned_error;
    }

    geographic_msgs::GeoPoint localToGlobalGoal(const geographic_msgs::GeoPoint& curr_pos, const geometry_msgs::Point& rel_goal_pos)
    {
        // TODO: move to helper file later
        // Convert the anchor GPS to a geodesy UTM point
        geodesy::UTMPoint curr_utm;
        geodesy::fromMsg(curr_pos, curr_utm);

        // Create a new UTM point with the offset applied
        geodesy::UTMPoint utm_target;
        utm_target.northing  = curr_utm.northing + rel_goal_pos.x;
        utm_target.easting = curr_utm.easting + rel_goal_pos.y;
        utm_target.altitude = curr_utm.altitude + rel_goal_pos.z;
        utm_target.zone     = curr_utm.zone;
        utm_target.band     = curr_utm.band;

        // Convert back to GPS (lat, lon, alt)
        const geographic_msgs::GeoPoint target_gps = geodesy::toMsg(utm_target);

        return target_gps;
    }

    bool validMission(const osdk::MissionGoalConstPtr& goal)
    {
        // validate altitude isn't lower than the starting position
        const auto msg = ros::topic::waitForMessage<sensor_msgs::NavSatFix>("dji_osdk_ros/gps_position", nh_, ros::Duration(TOPIC_TIMEOUT_S));
        if (!msg)
        {
            // no position was recieved in the timeout
            ROS_ERROR("No GPS position was published in the timeout");
            return false;
        }
        const auto curr_pos = navSatFixtoGeoPoint(*msg);

        geographic_msgs::GeoPoint goal_pos;
        if (goal->relative)
        {
            goal_pos = localToGlobalGoal(curr_pos, goal->rel_goal_position);
        }
        else
        {
            goal_pos = goal->abs_goal_position;
        }

        goal_ned_error_ = getNEDError(curr_pos, goal_pos);

        const auto goal_distance = goal_ned_error_.norm();

        if (goal_distance > goal->max_distance)
        {
            ROS_ERROR_STREAM("The distance to the goal was greater than the allowable distance of " << std::to_string(goal->max_distance) << " meters");
            return false;
        }

        // check that the drone isn't moving down from  the current position
        // assume valid missions must go up to avoid crashing
        if (goal_ned_error_.z() > 0)
        {
            // down error is positive; drone wants to go down
            ROS_ERROR("A valid mission cannot make the drone move downwards!");
            return false;
        }
        return true;
    }

    void waypointReachedCallback(const actionlib::SimpleClientGoalState& state, const osdk::MoveToWaypointResultConstPtr& result)
    {
        // ROS_ERROR("IN WAYPOINY REACHED CB");
        // Check if the mission has been preempted
        // if (as_.isPreemptRequested() || !ros::ok())
        // {
        //     ROS_ERROR("Mission preempted during waypoint navigation");
        //     as_.setPreempted();
        //     ac_.cancelAllGoals();
        //     return;
        // }

        if (state == actionlib::SimpleClientGoalState::PREEMPTED)
        {
            ROS_ERROR("Waypoint navigation was preempted");
            if (as_.isActive())
            {
                result_.success = false;
                result_.message = "Mission was preempted during waypoint navigation";
                as_.setPreempted(result_, "Mission was preempted");
            }
            return;
        }

        if (!(state == actionlib::SimpleClientGoalState::SUCCEEDED && result->success))
        {
            ROS_ERROR("NICO CRY MORE");
            ROS_ERROR("Failed to reach waypoint %ld with state: %s",
                waypoint_idx_ + 1, state.toString().c_str());
            result_.success = false;
            as_.setAborted(result_, "Navigation failed");
            return;
        }

        ROS_ERROR("Reached waypoint %ld", waypoint_idx_ + 1);
        // Move to next waypoint
        waypoint_idx_++;
        feedback_.n_waypoint = waypoint_idx_ + 1;
        as_.publishFeedback(feedback_);
        navigateToNextWaypoint();
    }

    void activeCallback()
    {
        // fill for starting to navigate to a new waypoint
    }

    // void feedbackCallback()
    // {
    //     // this is the key; check for preemptions of this action server as it runs it's action client
    //     if (as_.isPreemptRequested() || !ros::ok())
    //     {
    //         ROS_ERROR("Mission preempted");
    //         as_.setPreempted();
    //         ac_.cancelAllGoals();
    //         return;
    //     }
    //     // might be redundant below and too much span; consider removing
    //     feedback_.n_waypoint = waypoint_idx_+1;
    //     as_.publishFeedback(feedback_);
    // }

    void navigateToNextWaypoint()
    {
        // Check if we should continue
        // if (as_.isPreemptRequested() || !ros::ok())
        // {
        //     ROS_ERROR("Mission preempted");
        //     as_.setPreempted();
        //     ac_.cancelAllGoals();
        //     return;
        // }
        if (waypoint_idx_ >= waypoints_.size())
        {
            // TODO: consider adding in landing later
            ROS_ERROR("Mission completed successfully");
            result_.success = true;
            as_.setSucceeded(result_);
            return;
        }

        const auto waypoint = waypoints_[waypoint_idx_];
        osdk::MoveToWaypointGoal goal;
        goal.relative = true;
        goal.rel_goal_position.x = waypoint.x;
        goal.rel_goal_position.y = waypoint.y;
        goal.rel_goal_position.z = waypoint.z;

        ac_.sendGoal(goal, std::bind(&MissionPlannerActionServer::waypointReachedCallback, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MissionPlannerActionServer::activeCallback, this));
            // std::bind(&MissionPlannerActionServer::feedbackCallback, this));
    }

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<osdk::MissionAction> as_;
    std::string action_name_;

    osdk::MissionFeedback feedback_;
    osdk::MissionResult result_;

    actionlib::SimpleActionClient<osdk::MoveToWaypointAction> ac_;

    ros::ServiceClient task_control_client_;
    ros::ServiceClient enable_horizon_avoid_client_;
    ros::ServiceClient enable_upward_avoid_client_;
    ros::ServiceClient get_avoid_enable_client_;
    ros::ServiceClient obtain_ctrl_authority_client_;

    Eigen::Vector3d goal_ned_error_;

    size_t waypoint_idx_ { 0 };
    std::vector<geometry_msgs::Point> waypoints_;

    // primitive mission planner information
    const int N_WAYPOINTS { 3 };
    // the flight altitude of the drone
    const double FLYING_ALTITUDE_M { 20.0 };
    const double SINLGE_WAYPOINT_TIMEOUT_S { 60.0 };
    const double TOPIC_TIMEOUT_S { 5.0 };
};
    

} // impulse_control

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission_planner_action_server");
    impulse_control::MissionPlannerActionServer server("mission_planner");
    ros::spin();
    return 0;
}
