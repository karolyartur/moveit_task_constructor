/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein
   Desc:   A demo to show MoveIt Task Constructor in action
*/

// ROS
#include <ros/ros.h>

// MTC pick/place demo implementation
#include <moveit_task_constructor_demo/pick_place_task.h>
#include <moveit_task_constructor_msgs/Benchmark.h>

#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_ros/transform_broadcaster.h>

constexpr char LOGNAME[] = "moveit_task_constructor_demo";

class benchmark_server
{
	public:

	ros::ServiceServer service;
	ros::NodeHandle nh;
	ros::NodeHandle snh;
	moveit_task_constructor_demo::PickPlaceTask pick_place_task;

	// PARAMS:
	int object_type = shape_msgs::SolidPrimitive::CYLINDER;
	int number_of_solutions_per_run = 1;

	benchmark_server();

	void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object) {
		if (!psi.applyCollisionObject(object))
			throw std::runtime_error("Failed to spawn object: " + object.id);
	}

	moveit_msgs::CollisionObject createObject(uint8_t object_type) {
		ros::NodeHandle pnh("~");
		std::string object_name, object_reference_frame;
		std::vector<double> object_dimensions;
		geometry_msgs::Pose pose;
		std::size_t error = 0;
		error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", object_name);
		error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
		error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_dimensions", object_dimensions);
		error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_pose", pose);
		rosparam_shortcuts::shutdownIfError(LOGNAME, error);

		moveit_msgs::CollisionObject object;
		object.id = object_name;
		object.header.frame_id = object_reference_frame;
		object.primitives.resize(1);
		object.primitives[0].type = object_type;
		object.primitives[0].dimensions = object_dimensions;
		pose.position.z += 0.5 * object_dimensions[0];
		object.primitive_poses.push_back(pose);
		return object;
	}

	moveit_msgs::CollisionObject createBox(double dimensions[3], geometry_msgs::Pose pose) {
		std::vector<double> box_dimensions;
		box_dimensions.assign(dimensions, dimensions+3);
		moveit_msgs::CollisionObject object;
		object.id = "box1";
		object.header.frame_id = "world";
		object.primitives.resize(1);
		object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
		object.primitives[0].dimensions = box_dimensions;
		object.primitive_poses.push_back(pose);
		return object;
	}

	bool benchmark(moveit_task_constructor_msgs::Benchmark::Request &req, moveit_task_constructor_msgs::Benchmark::Response &res);
};

benchmark_server::benchmark_server(): pick_place_task("pick_place_task", benchmark_server::nh){	

	ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service
	moveit::planning_interface::PlanningSceneInterface psi;
	ros::NodeHandle pnh("~");

	spawnObject(psi, createObject(object_type));

	geometry_msgs::Pose pose;
	double dimensions[3] = {0.25, 0.25, 0.5};
	pose.position.x = 0.5;
	pose.position.y = 0.0;
	pose.position.z += 0.5 * dimensions[2];

	spawnObject(psi, createBox(dimensions, pose));

	service = benchmark_server::snh.advertiseService("benchmark", &benchmark_server::benchmark, this);
	ROS_INFO("end");
}

bool benchmark_server::benchmark(moveit_task_constructor_msgs::Benchmark::Request &req, moveit_task_constructor_msgs::Benchmark::Response &res)
{
	std::vector<float> durations;
	std::vector<ros::Time> start_times;
	benchmark_server::pick_place_task.loadParameters();
	benchmark_server::pick_place_task.init(object_type);
	std::vector<moveit_task_constructor_msgs::Solution> solutions;
	moveit_task_constructor_msgs::Solution sol;
	for (int i = 0; i < req.number_of_runs; i++)
	{
		ros::WallTime start_time = ros::WallTime::now();
		if (benchmark_server::pick_place_task.plan(number_of_solutions_per_run)) {
			ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
			sol = pick_place_task.getSolution();
			solutions.push_back(sol);
		} else {
			ROS_INFO_NAMED(LOGNAME, "Planning failed");
		}
		double duration = (ros::WallTime::now() - start_time).toSec();
		durations.push_back(duration);
		start_times.push_back(ros::Time(start_time.sec, start_time.nsec));
	}
	res.durations = durations;
	res.start_times = start_times;
	res.solutions = solutions;
}

int main(int argc, char** argv) {
	ROS_INFO_NAMED(LOGNAME, "Init moveit_task_constructor_demo");
	ros::init(argc, argv, "moveit_task_constructor_demo");
	benchmark_server s;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Keep introspection alive
	ros::waitForShutdown();
	return 0;
}
