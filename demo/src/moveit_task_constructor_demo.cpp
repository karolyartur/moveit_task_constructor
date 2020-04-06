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
#include <moveit_task_constructor_msgs/BenchmarkResult.h>

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

	moveit::planning_interface::PlanningSceneInterface psi;

	std::vector<std::string> object_types;
	std::vector<uint8_t> constraints;
	std::vector<double> waypoints;

	// PARAMS:
	int number_of_solutions_per_run = 1;
	int timeout = 30;

	benchmark_server();

	void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object) {
		if (!psi.applyCollisionObject(object))
			throw std::runtime_error("Failed to spawn object: " + object.id);
	}

	moveit_msgs::CollisionObject createObject(uint8_t object_type) {
		ros::NodeHandle pnh("~");
		std::string object_name, object_reference_frame;
		std::vector<double> box_dimensions;
		std::vector<double> cylinder_dimensions;
		std::vector<double> sphere_dimensions;
		geometry_msgs::Pose pose;
		std::size_t error = 0;
		error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", object_name);
		error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
		error += !rosparam_shortcuts::get(LOGNAME, pnh, "box_dimensions", box_dimensions);
		error += !rosparam_shortcuts::get(LOGNAME, pnh, "cylinder_dimensions", cylinder_dimensions);
		error += !rosparam_shortcuts::get(LOGNAME, pnh, "sphere_dimensions", sphere_dimensions);
		error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_pose", pose);
		rosparam_shortcuts::shutdownIfError(LOGNAME, error);

		moveit_msgs::CollisionObject object;
		object.id = object_name;
		object.header.frame_id = object_reference_frame;
		object.primitives.resize(1);
		object.primitives[0].type = object_type;
		if (object_type == shape_msgs::SolidPrimitive::BOX){
			object.primitives[0].dimensions = box_dimensions;
			pose.position.z += 0.5 * box_dimensions[2];
		}
		else if (object_type == shape_msgs::SolidPrimitive::CYLINDER){
			object.primitives[0].dimensions = cylinder_dimensions;
			pose.position.z += 0.5 * cylinder_dimensions[0];
		}
		else if (object_type == shape_msgs::SolidPrimitive::SPHERE){
			object.primitives[0].dimensions = sphere_dimensions;
			pose.position.z += sphere_dimensions[0];
		}
		object.primitive_poses.push_back(pose);
		return object;
	}

	moveit_msgs::CollisionObject createBox() {
		ros::NodeHandle pnh("~");
		std::vector<double> obstacle_dimensions;
		geometry_msgs::Pose obstacle_pose;
		std::size_t error = 0;
		error += !rosparam_shortcuts::get(LOGNAME, pnh, "obstacle_dimensions", obstacle_dimensions);
		error += !rosparam_shortcuts::get(LOGNAME, pnh, "obstacle_pose", obstacle_pose);
		rosparam_shortcuts::shutdownIfError(LOGNAME, error);

		moveit_msgs::CollisionObject object;
		object.id = "box";
		object.header.frame_id = "world";
		object.primitives.resize(1);
		object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
		object.primitives[0].dimensions = obstacle_dimensions;
		obstacle_pose.position.z += 0.5 * obstacle_dimensions[2];
		object.primitive_poses.push_back(obstacle_pose);
		return object;
	}

	bool benchmark(moveit_task_constructor_msgs::Benchmark::Request &req, moveit_task_constructor_msgs::Benchmark::Response &res);
};

benchmark_server::benchmark_server(): pick_place_task("pick_place_task", benchmark_server::nh){	
	ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service

	ros::NodeHandle pnh("~");

	std::size_t error = 0;
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_types", object_types);

	constraints.push_back(true);
	// constraints.push_back(false);

	//waypoints.push_back(0.02);
    waypoints.push_back(0.05);
	//waypoints.push_back(0.0);

	rosparam_shortcuts::shutdownIfError(LOGNAME, error);

	service = benchmark_server::snh.advertiseService("benchmark", &benchmark_server::benchmark, this);
	ROS_INFO("Service started");
}

bool benchmark_server::benchmark(moveit_task_constructor_msgs::Benchmark::Request &req, moveit_task_constructor_msgs::Benchmark::Response &res)
{
	ros::NodeHandle pnh("~");
	std::vector<float> durations;
	std::vector<ros::Time> start_times;
	int object_type;
	
	std::vector<moveit_task_constructor_msgs::BenchmarkResult> results;
	for (double waypoint_distance : waypoints){
		pnh.setParam("/move_group/ompl/maximum_waypoint_distance", waypoint_distance);
		for (std::string ot:object_types){
			moveit_task_constructor_msgs::BenchmarkResult benchmarkresult;

			if (ot == "BOX"){
				object_type = shape_msgs::SolidPrimitive::BOX;
			}
			else if (ot == "CYLINDER"){
				object_type = shape_msgs::SolidPrimitive::CYLINDER;
			}
			else if (ot == "SPHERE"){
				object_type = shape_msgs::SolidPrimitive::SPHERE;
			}
			for (bool is_path_constrained : constraints){
				spawnObject(psi, createObject(object_type));

				spawnObject(psi, createBox());

				benchmark_server::pick_place_task.loadParameters();
				benchmark_server::pick_place_task.init(object_type, is_path_constrained);
				std::vector<moveit_task_constructor_msgs::Solution> solutions;
				std::vector<uint8_t> successes;
				std::vector<std::string> config;
				if (waypoint_distance == 0.02){
					config.push_back("dense");
				}
				else if (waypoint_distance == 0.05){
					config.push_back("medium");
				}
				else if (waypoint_distance == 0.0){
					config.push_back("sparse");
				}
				config.push_back(ot);
				if (is_path_constrained){
					config.push_back("True");
				}
				else{
					config.push_back("False");
				}
				moveit_task_constructor_msgs::Solution sol;
				moveit_task_constructor_msgs::Solution empty_sol;
				bool success = false;
				for (int i = 0; i < req.number_of_runs; i++)
				{
					ros::WallTime start_time = ros::WallTime::now();
					success = benchmark_server::pick_place_task.plan(number_of_solutions_per_run, timeout);
					if (success) {
						ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
						sol = pick_place_task.getSolution();
						solutions.push_back(sol);
					} else {
						ROS_INFO_NAMED(LOGNAME, "Planning failed");
						solutions.push_back(empty_sol);
					}
					double duration = (ros::WallTime::now() - start_time).toSec();
					successes.push_back(success);
					durations.push_back(duration);
					start_times.push_back(ros::Time(start_time.sec, start_time.nsec));
				}
				benchmarkresult.durations = durations;
				benchmarkresult.start_times = start_times;
				benchmarkresult.success = successes;
				benchmarkresult.solutions = solutions;
				benchmarkresult.config = config;
				durations.clear();
				//successes.clear();
				//solutions.clear();
				//config.clear();

				results.push_back(benchmarkresult);
			}
		}
	}
	res.benchmarks = results;
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
