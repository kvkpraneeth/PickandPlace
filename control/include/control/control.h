#ifndef CONTROL_H
#define CONTROL_H

#include "gazebo_msgs/GetModelState.h"
#include "geometry_msgs/Pose.h"
#include "moveit/robot_model/joint_model_group.h"
#include "ros/forwards.h"
#include "ros/node_handle.h"
#include "ros/ros.h"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/AttachedCollisionObject.h"
#include "moveit_msgs/CollisionObject.h"

#include <memory>
#include "ros/service_client.h"
#include "ros/timer.h"

class control
{
    //Constructor.
    public: control(ros::NodeHandle& nh_);

    //Destructor.
    public: ~control();

    //Environment Literals.
    private: const std::string armPlanningGroup = "lemons_arm";
    private: const std::string handPlanningGroup = "lemons_hand";

    //Moveit Declaration.
    private: moveit::planning_interface::MoveGroupInterface* moveGroupInterface; 
    private: moveit::planning_interface::PlanningSceneInterface* planningSceneInterface;
    private: const moveit::core::JointModelGroup* jointModelGroup;
    private: moveit::planning_interface::MoveGroupInterface::Plan movementPlan;
    private: moveit_msgs::CollisionObject objToAttach;

    //Goal Variables.
    public: geometry_msgs::Pose goalPose1;
            gazebo_msgs::GetModelState modelState;
            gazebo_msgs::GetModelState modelState1;

    //Worker Functions.
    public: void moveToGoal(const ros::TimerEvent& event);
    public: void updateParam(const ros::TimerEvent& event);

    //Timer.
    private: ros::Timer timer;
    private: ros::Timer paramTimer;

    //NodeHandle.
    private: ros::NodeHandle nnh_;

    //Service Client.
    private: ros::ServiceClient client;

};

#endif
