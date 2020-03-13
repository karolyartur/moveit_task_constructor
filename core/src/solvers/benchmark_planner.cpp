#include <moveit/task_constructor/solvers/benchmark_planner.h>
#include <moveit/task_constructor/task.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

namespace moveit {
namespace task_constructor {
namespace solvers {

BenchmarkPlanner::BenchmarkPlanner(): benchmark(), opts(ros::this_node::getName()) {
	auto& p = properties();
	p.declare<std::string>("planner", "", "planner id");

	p.declare<uint>("num_planning_attempts", 1u, "number of planning attempts");
	p.declare<moveit_msgs::WorkspaceParameters>("workspace_parameters", moveit_msgs::WorkspaceParameters(),
	                                            "allowed workspace of mobile base?");

	p.declare<double>("goal_joint_tolerance", 1e-4, "tolerance for reaching joint goals");
	p.declare<double>("goal_position_tolerance", 1e-4, "tolerance for reaching position goals");
	p.declare<double>("goal_orientation_tolerance", 1e-4, "tolerance for reaching orientation goals");

	p.declare<bool>("display_motion_plans", false,
	                "publish generated solutions on topic " + planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC);
	p.declare<bool>("publish_planning_requests", false,
	                "publish motion planning requests on topic " +
	                    planning_pipeline::PlanningPipeline::MOTION_PLAN_REQUEST_TOPIC);
	std::vector<std::string> planning_pipelines;
	opts.getPlanningPipelineNames(planning_pipelines);
	benchmark.initialize(planning_pipelines);
}

void BenchmarkPlanner::init(const core::RobotModelConstPtr& robot_model) {
	planner_ = Task::createPlanner(robot_model);

	planner_->displayComputedMotionPlans(properties().get<bool>("display_motion_plans"));
	planner_->publishReceivedRequests(properties().get<bool>("publish_planning_requests"));
}

void initMotionPlanRequest_2(moveit_msgs::MotionPlanRequest& req, const PropertyMap& p,
                           const moveit::core::JointModelGroup* jmg, double timeout) {
	req.group_name = jmg->getName();
	req.planner_id = p.get<std::string>("planner");
	req.allowed_planning_time = timeout;
	req.start_state.is_diff = true;  // we don't specify an extra start state

	req.num_planning_attempts = p.get<uint>("num_planning_attempts");
	req.max_velocity_scaling_factor = p.get<double>("max_velocity_scaling_factor");
	req.max_acceleration_scaling_factor = p.get<double>("max_acceleration_scaling_factor");
	req.workspace_parameters = p.get<moveit_msgs::WorkspaceParameters>("workspace_parameters");
}

bool BenchmarkPlanner::plan(const planning_scene::PlanningSceneConstPtr& from,
                           const planning_scene::PlanningSceneConstPtr& to, const moveit::core::JointModelGroup* jmg,
                           double timeout, robot_trajectory::RobotTrajectoryPtr& result,
                           const moveit_msgs::Constraints& path_constraints) {
	const auto& props = properties();
	moveit_msgs::MotionPlanRequest req;
	initMotionPlanRequest_2(req, props, jmg, timeout);

	req.goal_constraints.resize(1);
	req.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(to->getCurrentState(), jmg,
	                                                                          props.get<double>("goal_joint_tolerance"));
	req.path_constraints = path_constraints;

	::planning_interface::MotionPlanResponse res;
	bool success = planner_->generatePlan(from, req, res);
	result = res.trajectory_;
	return success;
}

bool BenchmarkPlanner::plan(const planning_scene::PlanningSceneConstPtr& from, const moveit::core::LinkModel& link,
                           const Eigen::Isometry3d& target_eigen, const moveit::core::JointModelGroup* jmg,
                           double timeout, robot_trajectory::RobotTrajectoryPtr& result,
                           const moveit_msgs::Constraints& path_constraints) {
	const auto& props = properties();
	moveit_msgs::MotionPlanRequest req;
	initMotionPlanRequest_2(req, props, jmg, timeout);

	geometry_msgs::PoseStamped target;
	target.header.frame_id = from->getPlanningFrame();
	tf::poseEigenToMsg(target_eigen, target.pose);

	req.goal_constraints.resize(1);
	req.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(
	    link.getName(), target, props.get<double>("goal_position_tolerance"),
	    props.get<double>("goal_orientation_tolerance"));
	req.path_constraints = path_constraints;

	::planning_interface::MotionPlanResponse res;
	bool success = planner_->generatePlan(from, req, res);
	result = res.trajectory_;
	return success;
}
}
}
}
