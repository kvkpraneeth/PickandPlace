#include "control/control.h"
#include "geometry_msgs/PoseStamped.h"
#include "moveit/kinematics_base/kinematics_base.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/robot_model/joint_model_group.h"
#include "moveit_msgs/Grasp.h"
#include "ros/duration.h"
#include "ros/timer.h"
#include "shape_msgs/SolidPrimitive.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include <stdexcept>
#include "gazebo_msgs/GetModelState.h"

control::control(ros::NodeHandle &nh_)
{
    this->moveGroupInterface = new moveit::planning_interface::MoveGroupInterface(this->armPlanningGroup);
    this->planningSceneInterface = new moveit::planning_interface::PlanningSceneInterface();
    this->jointModelGroup = (new moveit::planning_interface::MoveGroupInterface(this->armPlanningGroup))->getCurrentState()->getJointModelGroup(this->armPlanningGroup);
    this->nnh_ = nh_;
    this->timer = nh_.createTimer(ros::Duration(2.0), &control::moveToGoal, this);
    this->paramTimer = nh_.createTimer(ros::Duration(0.1), &control::updateParam, this);

    this->moveGroupInterface->setEndEffectorLink("gripper_link");
    this->moveGroupInterface->setPoseReferenceFrame("arm0");
    this->moveGroupInterface->setNumPlanningAttempts(30);
    // this->moveGroupInterface->setGoalJointTolerance(0.0000001);
    this->moveGroupInterface->setGoalOrientationTolerance(0.0000001);
    this->moveGroupInterface->setGoalPositionTolerance(0.0000001);

    this->client = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    this->modelState.request.model_name = "obj";
    this->modelState.request.relative_entity_name = "arm0";

    this->modelState1.request.model_name = "obj";
    this->modelState1.request.relative_entity_name = "gripper_link";

}

control::~control()
{
    delete moveGroupInterface;
    delete planningSceneInterface;
    delete jointModelGroup;
}

void control::moveToGoal(const ros::TimerEvent& event)
{

    moveit::core::RobotStatePtr currentState = moveGroupInterface->getCurrentState();

    //The Below GoalPose is in Cartesian Space. arm0->gripper_link;

    // tf2::Quaternion quat;

    // double r=0,p=0,y=0;
    //
    // this->nnh_.getParam("r", r);
    // this->nnh_.getParam("p", p);
    // this->nnh_.getParam("yaw", y);
    //
    // quat.setRPY(r, p, y);
    //
    // goalPose1.orientation.w = quat.w();
    // goalPose1.orientation.x = quat.x();
    // goalPose1.orientation.y = quat.y();
    // goalPose1.orientation.z = quat.z();

    this->nnh_.getParam("qx", goalPose1.orientation.x);
    this->nnh_.getParam("qy", goalPose1.orientation.y);
    this->nnh_.getParam("qz", goalPose1.orientation.z);
    this->nnh_.getParam("qw", goalPose1.orientation.w);
    this->nnh_.getParam("x", goalPose1.position.x);
    this->nnh_.getParam("y", goalPose1.position.y);
    this->nnh_.getParam("z", goalPose1.position.z);

    // goalPose1.position.z -= 0.2;
    ROS_INFO("%f %f %f %f", goalPose1.orientation.x, goalPose1.orientation.y, goalPose1.orientation.z, goalPose1.orientation.w);
    ROS_INFO("%f %f %f", goalPose1.position.x, goalPose1.position.y, goalPose1.position.z);

    moveGroupInterface->setApproximateJointValueTarget(goalPose1, "gripper_link");

    bool success = (moveGroupInterface->plan(movementPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if(success)
    {
        ROS_INFO("%s", "SUCCESS in Planning");
        
        moveGroupInterface->move();
    }

    if(!success)
    {
        ROS_INFO("%s", "Failure in Planning.");
    }

    /*    
    Collision Objects to be added and further not integrated in grasping, as it is too complicated for the given Deadline.
    Instead using brute force lifting after experimenting and pushing gazebo's physics-> still glitchy.

    std::vector<moveit_msgs::CollisionObject> collisionObjects;
    collisionObjects.resize(1);

    collisionObjects[0].id = "obj";
    collisionObjects[0].header.frame_id = "arm0";

    collisionObjects[0].primitives.resize(1);
    collisionObjects[0].primitives[0].type = collisionObjects[0].primitives[0].BOX;
    collisionObjects[0].primitives[0].dimensions.resize(3);

    collisionObjects[0].primitives[0].dimensions[0] = 0.1;
    collisionObjects[0].primitives[0].dimensions[1] = 0.1;
    collisionObjects[0].primitives[0].dimensions[1] = 0.1;

    collisionObjects[0].primitive_poses.resize(1);
    collisionObjects[0].primitive_poses[0].position.x = 1.796401;
    collisionObjects[0].primitive_poses[0].position.y = -0.178903;
    collisionObjects[0].primitive_poses[0].position.z = 0.200416;
    collisionObjects[0].primitive_poses[0].orientation.w = 1.0;

    std::vector<moveit_msgs::Grasp> grasps;

    grasps.resize(1);

    grasps[0].grasp_pose.header.frame_id = "arm0";
    grasps[0].grasp_pose.pose.orientation.w = -0.48438;
    grasps[0].grasp_pose.pose.orientation.x = 0.51475;
    grasps[0].grasp_pose.pose.orientation.y = 0.4847;
    grasps[0].grasp_pose.pose.orientation.z = 0.5151;
    grasps[0].grasp_pose.pose.position.x = 1.7941;
    grasps[0].grasp_pose.pose.position.y = -0.10936;
    grasps[0].grasp_pose.pose.position.z = 0.25;

    grasps[0].pre_grasp_approach.direction.header.frame_id = "arm0";
    grasps[0].pre_grasp_approach.direction.vector.z = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.05;
    grasps[0].pre_grasp_approach.desired_distance = 0.02;

    grasps[0].post_grasp_retreat.direction.header.frame_id = "arm0";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.03;
    grasps[0].post_grasp_retreat.desired_distance = 0.04;

    moveGroupInterface->pick("obj", grasps);

    */

}

void control::updateParam(const ros::TimerEvent &event)
{
    if(this->client.call(this->modelState))
    {
        
        this->nnh_.setParam("x", modelState.response.pose.position.x);
        this->nnh_.setParam("y", modelState.response.pose.position.y);
        this->nnh_.setParam("z", modelState.response.pose.position.z);

        this->nnh_.setParam("qx", modelState.response.pose.orientation.x);
        this->nnh_.setParam("qy", modelState.response.pose.orientation.y);
        this->nnh_.setParam("qz", modelState.response.pose.orientation.z);
        this->nnh_.setParam("qw", modelState.response.pose.orientation.w);

        // this->nnh_.setParam("xr", modelState1.response.pose.position.x);
        // this->nnh_.setParam("yr", modelState1.response.pose.position.y);
        // this->nnh_.setParam("zr", modelState1.response.pose.position.z);
        //
        // this->nnh_.setParam("qxr", modelState1.response.pose.orientation.x);
        // this->nnh_.setParam("qyr", modelState1.response.pose.orientation.y);
        // this->nnh_.setParam("qzr", modelState1.response.pose.orientation.z);
        // this->nnh_.setParam("qwr", modelState1.response.pose.orientation.w);
        //
    }

}
