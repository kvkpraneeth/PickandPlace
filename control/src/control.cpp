#include "control/control.h"
#include "moveit/kinematics_base/kinematics_base.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/robot_model/joint_model_group.h"
#include "moveit_msgs/Grasp.h"
#include "shape_msgs/SolidPrimitive.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include <stdexcept>

control::control()
{
    this->moveGroupInterface = new moveit::planning_interface::MoveGroupInterface(this->armPlanningGroup);
    this->planningSceneInterface = new moveit::planning_interface::PlanningSceneInterface();
    this->jointModelGroup = (new moveit::planning_interface::MoveGroupInterface(this->armPlanningGroup))->getCurrentState()->getJointModelGroup(this->armPlanningGroup);
}

control::~control()
{
    delete moveGroupInterface;
    delete planningSceneInterface;
    delete jointModelGroup;
}

void control::moveToGoal()
{

    moveit::core::RobotStatePtr currentState = moveGroupInterface->getCurrentState();

    //The Below GoalPose is in Cartesian Space. arm0->gripper_link;
    goalPose1.orientation.w = -0.35987;
    goalPose1.orientation.x = -0.60853;
    goalPose1.orientation.y = -0.35958;
    goalPose1.orientation.z = 0.60901;
    goalPose1.position.x = -0.8446;
    goalPose1.position.y = -0.60369;
    goalPose1.position.z = 0.29936;

    moveGroupInterface->setPoseTarget(goalPose1);

    bool success = (moveGroupInterface->plan(movementPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if(!success)
    {
        throw std::runtime_error("Failure in Planning");
    }
    
    if(success)
    {
        ROS_INFO("%s", "SUCCESS in Planning");
    }

    moveGroupInterface->execute(movementPlan);

    /*    
    Collision Objects to be added and further not integrated in grasping, as it is too complicated for the given Deadline.
    Instead using a script to generate a downward force on the gripper of F = mass*-9.8 and spawning and deleting the object.
    */


    // std::vector<moveit_msgs::CollisionObject> collisionObjects;
    // collisionObjects.resize(1);
    //
    // collisionObjects[0].id = "obj";
    // collisionObjects[0].header.frame_id = "arm0";
    //
    // collisionObjects[0].primitives.resize(1);
    // collisionObjects[0].primitives[0].type = collisionObjects[0].primitives[0].BOX;
    // collisionObjects[0].primitives[0].dimensions.resize(3);
    //
    // collisionObjects[0].primitives[0].dimensions[0] = 0.1;
    // collisionObjects[0].primitives[0].dimensions[1] = 0.1;
    // collisionObjects[0].primitives[0].dimensions[1] = 0.1;
    //
    // collisionObjects[0].primitive_poses.resize(1);
    // collisionObjects[0].primitive_poses[0].position.x = 1.796401;
    // collisionObjects[0].primitive_poses[0].position.y = -0.178903;
    // collisionObjects[0].primitive_poses[0].position.z = 0.200416;
    //
    // std::vector<moveit_msgs::Grasp> grasps;
    //
    // grasps.resize(1);
    //
    // grasps[0].grasp_pose.header.frame_id = "arm3";
    // tf2::Quaternion orientation;
    // orientation.setRPY(0.0, 0.0, 0.0);
    // grasps[0].grasp_pose.pose.orientation.w = orientation.w();
    // grasps[0].grasp_pose.pose.orientation.x = orientation.x();
    // grasps[0].grasp_pose.pose.orientation.y = orientation.y();
    // grasps[0].grasp_pose.pose.orientation.z = orientation.z();
    // grasps[0].grasp_pose.pose.position.x = 0;
    // grasps[0].grasp_pose.pose.position.y = 0;
    // grasps[0].grasp_pose.pose.position.z = -0.1;

    // grasps[0].pre_grasp_approach.direction.vector.z = 1.0;
    // grasps[0].pre_grasp_approach.min_distance = 0.03;
    // grasps[0].pre_grasp_approach.desired_distance = 0.02;
    //
    // grasps[0].post_grasp_retreat.direction.header.frame_id = "arm3";
    // grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    // grasps[0].post_grasp_retreat.min_distance = 0.03;
    // grasps[0].post_grasp_retreat.desired_distance = 0.04;

    // moveGroupInterface->pick("obj", grasps);

}
