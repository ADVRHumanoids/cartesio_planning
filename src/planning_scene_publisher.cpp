/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Michael Lautman */

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>


ros::ServiceClient srv;

void attached_collision_object_cb(const moveit_msgs::AttachedCollisionObjectConstPtr msg)
{
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object = *msg;

    // Add an object into the environment
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Add the object into the environment by adding it to
    // the set of collision objects in the "world" part of the
    // planning scene. Note that we are using only the "object"
    // field of the attached_object message here.
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene.is_diff = true;


    srv.waitForExistence();

    moveit_msgs::ApplyPlanningScene msg2;
    msg2.request.scene = planning_scene;

    srv.call(msg2);
}


void collision_object_cb(const moveit_msgs::CollisionObjectConstPtr msg)
{
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.object = *msg;

    // Add an object into the environment
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Add the object into the environment by adding it to
    // the set of collision objects in the "world" part of the
    // planning scene. Note that we are using only the "object"
    // field of the attached_object message here.
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;


    srv.waitForExistence();

    moveit_msgs::ApplyPlanningScene msg2;
    msg2.request.scene = planning_scene;

    srv.call(msg2);
}

void octomap_collision_object_cb(const octomap_msgs::OctomapWithPose msg)
{
    octomap_msgs::OctomapWithPose attached_object;
    attached_object = msg;

    // Add an object into the environment
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Add the object into the environment by adding it to
    // the set of collision objects in the "world" part of the
    // planning scene. Note that we are using only the "object"
    // field of the attached_object message here.
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.octomap = attached_object;
    planning_scene.is_diff = true;


    srv.waitForExistence();

    moveit_msgs::ApplyPlanningScene msg2;
    msg2.request.scene = planning_scene;

    srv.call(msg2);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planning_scene_ros_api_tutorial");

    ros::NodeHandle node_handle, nhpr("~");


    ros::Subscriber sub = node_handle.subscribe("planner/collision_objects", 1000, collision_object_cb);
    ros::Subscriber sub2 = node_handle.subscribe("planner/octomap_collision_objects", 1000, octomap_collision_object_cb);
    ros::Subscriber sub3 = node_handle.subscribe("planner/attached_collision_objects", 1000, attached_collision_object_cb);


    srv = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("planner/apply_planning_scene");


    ros::spin();

}
