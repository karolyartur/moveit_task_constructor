 #!/usr/bin/env python

import rospy
import moveit_task_constructor_msgs.srv
import numpy as np

from math import pi

class compute_benchmark():

    def __init__(self, num_runs):
        self._num_runs = num_runs
        rospy.wait_for_service('benchmark')
        self._service_client = rospy.ServiceProxy('benchmark', moveit_task_constructor_msgs.srv.Benchmark)

    def run_benchmark(self):
        results = self._service_client(self._num_runs)
        return results


if __name__ == "__main__":
    benchmark = compute_benchmark(1)
    results = benchmark.run_benchmark()
    durations = results.durations
    solutions = results.solutions
    
    sub_trajectories = [solution.sub_trajectory for solution in solutions]
    costs = []
    trajectories = []
    for sub_trajectory_list in sub_trajectories:
        subcosts = [solution_trajectory.info.cost for solution_trajectory in sub_trajectory_list]
        subtrajectories = [solution_trajectory.trajectory for solution_trajectory in sub_trajectory_list]
        costs.append(subcosts)
        trajectories.append(subtrajectories)

    j = 0
    trajectory_length = 0
    trajectory_times = []
    for sol in trajectories:
        print('\n new solution \n')
        moving_times = []
        for trajectory in sol:
            print('\n === \n')
            for i in range(len(trajectory.joint_trajectory.points)-1):
                prev = trajectory.joint_trajectory.points[i-1]
                curr = trajectory.joint_trajectory.points[i]
                dist = np.absolute(np.array(list(prev.positions))-np.array(list(curr.positions)))
                j += 1
                print(prev.positions)
                print(j, dist)
            if trajectory.joint_trajectory.points:
                moving_times.append(trajectory.joint_trajectory.points[-1].time_from_start.to_sec())
            print(moving_times)

    print("Press enter to publish Solution")
    raw_input()
    rospy.init_node('mtc_tc_test', anonymous=True)
    rospy.sleep(1)
    pub = rospy.Publisher('/mtc_tutorial/solution', moveit_task_constructor_msgs.msg.Solution, queue_size=10)
    print(solutions[0])
    pub.publish(solutions[0])
    while not rospy.is_shutdown():
        rospy.spin()