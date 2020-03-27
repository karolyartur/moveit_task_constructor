#pragma once

#include <moveit/benchmarks/BenchmarkOptions.h>
#include <moveit/benchmarks/MTCBenchmark.h>
#include <moveit/task_constructor/solvers/planner_interface.h>
#include <moveit/macros/class_forward.h>

namespace planning_pipeline {
MOVEIT_CLASS_FORWARD(PlanningPipeline)
}

namespace moveit {
namespace task_constructor {
namespace solvers {

MOVEIT_CLASS_FORWARD(BenchmarkPlanner)

/** Use MoveIt's PlanningPipeline to plan a trajectory between to scenes */
class BenchmarkPlanner : public PlannerInterface
{
public:
	BenchmarkPlanner();

	// void setPlannerId(const std::string& planner) { setProperty("planner", planner); }

	// void init(const moveit::core::RobotModelConstPtr& robot_model) override;

	// bool plan(const planning_scene::PlanningSceneConstPtr& from, const planning_scene::PlanningSceneConstPtr& to,
	//           const core::JointModelGroup* jmg, double timeout, robot_trajectory::RobotTrajectoryPtr& result,
	//           const moveit_msgs::Constraints& path_constraints = moveit_msgs::Constraints()) override;

	// bool plan(const planning_scene::PlanningSceneConstPtr& from, const moveit::core::LinkModel& link,
	//           const Eigen::Isometry3d& target, const core::JointModelGroup* jmg, double timeout,
	//           robot_trajectory::RobotTrajectoryPtr& result,
	//           const moveit_msgs::Constraints& path_constraints = moveit_msgs::Constraints()) override;

protected:
	planning_pipeline::PlanningPipelinePtr planner_;
	moveit_ros_benchmarks::BenchmarkOptions opts;
	moveit_ros_benchmarks::MTCBenchmark benchmark;
};
}
}
}
