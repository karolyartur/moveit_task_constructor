 #!/usr/bin/env python

import os
import rospy
import rospkg
import moveit_task_constructor_msgs.srv
import numpy as np
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import tf_conversions
import yaml

from math import pi

planner = 'CHOMP'

class compute_benchmark():

    def __init__(self, num_runs):
        self._num_runs = num_runs
        rospy.wait_for_service('benchmark')
        self._service_client = rospy.ServiceProxy('benchmark', moveit_task_constructor_msgs.srv.Benchmark)

    def run_benchmark(self):
        results = self._service_client(self._num_runs)
        return results


if __name__ == "__main__":

    benchmark = compute_benchmark(20)

    results = benchmark.run_benchmark()
        
    for result in results.benchmarks:

        print(result.config)

        if result.config[2] == 'True':
            constraint = 'constrained'
        else:
            constraint = 'unconstrained'

        filename = '_'.join(['raw',planner,result.config[0],constraint,result.config[1]])
        filename = filename + '.yaml'
        with open(filename, 'w') as file:
            documents = yaml.dump(results, file, default_flow_style=False)

        durations = result.durations
        solutions = result.solutions
        success = result.success

        sub_trajectories = [solution.sub_trajectory for solution in solutions]
        costs = []
        trajectories = []
        for sub_trajectory_list in sub_trajectories:
            subcosts = [solution_trajectory.info.cost for solution_trajectory in sub_trajectory_list]
            subtrajectories = [solution_trajectory.trajectory for solution_trajectory in sub_trajectory_list]
            costs.append(subcosts)
            trajectories.append(subtrajectories)

        j = 0
        trajectory_times = []
        discontinuous = []
        joint_space_trajectory_lengths = []
        for sol in trajectories:
            trajectory_length = np.zeros(7)
            moving_times = []
            maxdist = 0
            for trajectory in sol:
                for i in range(len(trajectory.joint_trajectory.points)-1):
                    prev = trajectory.joint_trajectory.points[i]
                    curr = trajectory.joint_trajectory.points[i+1]
                    dist = np.absolute(np.array(list(prev.positions))-np.array(list(curr.positions)))
                    j += 1
                    for joint in dist:
                        if joint > maxdist:
                            maxdist = joint
                    trajectory_length += dist
                if trajectory.joint_trajectory.points:
                    moving_times.append(trajectory.joint_trajectory.points[-1].time_from_start.to_sec())
                else:
                    moving_times.append(0)
            trajectory_times.append(moving_times)
            joint_space_trajectory_lengths.append(trajectory_length.tolist())
            if maxdist > pi/2:
                discontinuous.append(True)
            else:
                discontinuous.append(False)

        rospack = rospkg.RosPack()
        path = rospack.get_path('franka_description')
        path = os.path.join(path,'robots','panda_arm.xacro')
        robot_urdf = URDF.from_xml_string(rospy.get_param("/robot_description"))
        kdl_kin = KDLKinematics(robot_urdf, 'panda_link0', 'panda_link8')

        cartesian_traj_lengths = []
        for sol in trajectories:
            cartesian_traj_length = 0
            for traj in sol:
                for i in range(len(traj.joint_trajectory.points)-1):
                    prev = traj.joint_trajectory.points[i]
                    curr = traj.joint_trajectory.points[i+1]

                    prev_joints = prev.positions
                    curr_joints = curr.positions

                    prev_pose = tf_conversions.toMsg(tf_conversions.fromMatrix(kdl_kin.forward(prev_joints)))
                    curr_pose = tf_conversions.toMsg(tf_conversions.fromMatrix(kdl_kin.forward(curr_joints)))

                    prev_xyz = np.array([prev_pose.position.x, prev_pose.position.y, prev_pose.position.z])
                    curr_xyz = np.array([curr_pose.position.x, curr_pose.position.y, curr_pose.position.z])

                    distance = np.linalg.norm(prev_xyz - curr_xyz)
                    cartesian_traj_length += distance.item()
            cartesian_traj_lengths.append(cartesian_traj_length)

        movement_times = []
        for sol in trajectory_times:
            movement_times.append(sum(sol))

        filename = '_'.join(['test',planner,result.config[0],constraint,result.config[1]])
        filename = filename + '.yaml'

        benchmark_data = {'num_runs': benchmark._num_runs,
        'success': success,
        'planning_times': list(durations),
        'discontinuous_joint_trajectory': discontinuous,
        'trajectory_times': movement_times,
        'joint_trajectory_lengths': joint_space_trajectory_lengths,
        'cartesian_trajectory_lengths': cartesian_traj_lengths}

        with open(filename, 'w') as file:
            documents = yaml.dump(benchmark_data, file, default_flow_style=False)

    