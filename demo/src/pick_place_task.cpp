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

#include <moveit_task_constructor_demo/pick_place_task.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit_task_constructor_demo {
constexpr char LOGNAME[] = "pick_place_task";
PickPlaceTask::PickPlaceTask(const std::string& task_name, const ros::NodeHandle& nh)
  : nh_(nh), task_name_(task_name), execute_("execute_task_solution", true) {}

void PickPlaceTask::loadParameters() {
	/****************************************************
	 *                                                  *
	 *               Load Parameters                    *
	 *                                                  *
	 ***************************************************/
	ROS_INFO_NAMED(LOGNAME, "Loading task parameters");
	ros::NodeHandle pnh("~");

	// Planning group properties
	size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm_group_name", arm_group_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_group_name", hand_group_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "eef_name", eef_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_frame", hand_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "world_frame", world_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "grasp_frame_transform", grasp_frame_transform_);

	// Predefined pose targets
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_open_pose", hand_open_pose_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_close_pose", hand_close_pose_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm_home_pose", arm_home_pose_);

	// Target object
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", object_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "box_dimensions", box_dimensions);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "cylinder_dimensions", cylinder_dimensions);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "sphere_dimensions", sphere_dimensions);
	//support_surfaces_ = { surface_link_ };

	// Pick/Place metrics
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "place_pose", place_pose_);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

void PickPlaceTask::init(int object_type, bool is_path_constrained) {
	ROS_INFO_NAMED(LOGNAME, "Initializing task pipeline");
	const std::string object = object_name_;

	// Reset ROS introspection before constructing the new object
	// TODO(henningkayser): verify this is a bug, fix if possible
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());

	Task& t = *task_;
	t.stages()->setName(task_name_);
	t.loadRobotModel();

	// Sampling planner
	auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>();
	//pipeline_planner->setPlannerId("LBKPIECEkConfigDefault");
	pipeline_planner->setProperty("goal_joint_tolerance", 1e-5);

	// Set task properties
	t.setProperty("group", arm_group_name_);
	t.setProperty("eef", eef_name_);
	t.setProperty("hand", hand_group_name_);
	t.setProperty("hand_grasping_frame", hand_frame_);
	t.setProperty("ik_frame", hand_frame_);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	Stage* current_state = nullptr;  // Forward current_state on to grasp pose generator
	{
		auto _current_state = std::make_unique<stages::CurrentState>("current state");

		// Verify that object is not attached
		auto applicability_filter =
		    std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
		applicability_filter->setPredicate([object](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				comment = "object with id '" + object + "' is already attached and cannot be picked";
				return false;
			}
			return true;
		});

		current_state = applicability_filter.get();
		t.add(std::move(applicability_filter));
	}

	/****************************************************
	 *                                                  *
	 *               Move to Pick                       *
	 *                                                  *
	 ***************************************************/
	{  // Move-to pre-grasp
		auto stage = std::make_unique<stages::Connect>(
		    "move to pick", stages::Connect::GroupPlannerVector{ { arm_group_name_, pipeline_planner } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Pick Object                        *
	 *                                                  *
	 ***************************************************/
	Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator
	{
		auto grasp = std::make_unique<SerialContainer>("pick object");
		t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
		grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });


		/****************************************************
  ---- *               Generate Grasp Pose                *
		 ***************************************************/
		{
			// Sample grasp pose
			auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->properties().set("marker_ns", "grasp_pose");
			stage->setPreGraspPose(hand_open_pose_);
			stage->setObject(object);
			stage->setAngleDelta(M_PI / 12);
			stage->setMonitoredStage(current_state);  // Hook into current state
			stage->setObjectType(object_type);

			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(20);
			wrapper->setMinSolutionDistance(1.0);
			wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			grasp->insert(std::move(wrapper));
		}

		/****************************************************
  ---- *               Allow Collision (hand object)   *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
			stage->allowCollisions(
			    object, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
			    true);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  .... *               Attach Object                      *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
			stage->attachObject(object, hand_frame_);
			attach_object_stage = stage.get();
			grasp->insert(std::move(stage));
		}

		// Add grasp container to task
		t.add(std::move(grasp));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Place                             *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to place", stages::Connect::GroupPlannerVector{ { arm_group_name_, pipeline_planner } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		stage->isConstrained(is_path_constrained);
		t.add(std::move(stage));
	}

	/******************************************************
	 *                                                    *
	 *          Place Object                              *
	 *                                                    *
	 *****************************************************/
	{
		auto place = std::make_unique<SerialContainer>("place object");
		t.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
		place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });

		/******************************************************
  ---- *          Generate Place Pose                       *
		 *****************************************************/
		{
			// Generate Place Pose
			auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
			stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
			stage->properties().set("marker_ns", "place_pose");
			stage->setObject(object);

			// Set target pose
			geometry_msgs::PoseStamped p;
			p.header.frame_id = object_reference_frame_;
			p.pose = place_pose_;
			if (object_type == 1){
				p.pose.position.z +=  0.5 * box_dimensions[2];
			}
			else if (object_type == 2){
				p.pose.position.z += sphere_dimensions[0];
			}
			else if (object_type == 3){
				p.pose.position.z += 0.5 * cylinder_dimensions[0];
			}
			stage->setPose(p);
			stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(20);
			wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			place->insert(std::move(wrapper));
		}

		/******************************************************
  ---- *          Forbid collision (hand, object)        *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
			stage->allowCollisions(
			    object_name_,
			    t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(), false);
			place->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Detach Object                             *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
			stage->detachObject(object_name_, hand_frame_);
			place->insert(std::move(stage));
		}

		// Add place container to task
		t.add(std::move(place));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Home                              *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("move home", pipeline_planner);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		stage->setGoal(arm_home_pose_);
		stage->restrictDirection(stages::MoveTo::FORWARD);
		t.add(std::move(stage));
	}
}

bool PickPlaceTask::plan(int number_of_solutions_per_run, int timeout) {
	ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
	ros::NodeHandle pnh("~");

	try {
		task_->plan(number_of_solutions_per_run, timeout);
	} catch (InitStageException& e) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Initialization failed: " << e);
		return false;
	}
	if (task_->numSolutions() == 0) {
		ROS_ERROR_NAMED(LOGNAME, "Planning failed");
		return false;
	}
	return true;
}

bool PickPlaceTask::execute() {
	ROS_INFO_NAMED(LOGNAME, "Executing solution trajectory");
	moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
	task_->solutions().front()->fillMessage(execute_goal.solution);
	execute_.sendGoal(execute_goal);
	execute_.waitForResult();
	moveit_msgs::MoveItErrorCodes execute_result = execute_.getResult()->error_code;

	if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Task execution failed and returned: " << execute_.getState().toString());
		return false;
	}

	return true;
}

moveit_task_constructor_msgs::Solution PickPlaceTask::getSolution(){
	moveit_task_constructor_msgs::Solution solution;
	if (task_->numSolutions() != 0) {
		task_->solutions().front()->fillMessage(solution);
	}
	return solution;
}

}
