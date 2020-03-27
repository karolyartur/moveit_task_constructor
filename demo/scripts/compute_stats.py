import yaml
import rospkg
import os

import statistics

import matplotlib.pyplot as plt
import numpy as np

rospack = rospkg.RosPack()
#path = rospack.get_path('moveit_core')

filenames = [filename for (condition, filename) in zip([os.path.isfile(os.path.join('./', element)) for element in os.listdir('./')], os.listdir('./')) if condition == True]

to_yaml_list = []

data_to_plot = {'trajectory_length': [], 'planning_time': []}
labels_for_plot = []

for filename in filenames:
    if filename.split('_')[0] == 'test':
        planner_id = filename.split('_')[1]
        waypoint_density = filename.split('_')[2]
        path_constraint = filename.split('_')[3]
        object_type = filename.split('_')[4].split('.')[0]
        print(planner_id, waypoint_density, path_constraint, object_type)

        with open(filename) as f:
            benchmark_data = yaml.load(f)

        success = benchmark_data['success']
        discontinous = benchmark_data['discontinuous_joint_trajectory']

        mask = [v1 and v2 for (v1, v2) in zip([not disc for disc in discontinous], success)]

        ####
        success_percent = float(success.count(True))/len(success)

        #####
        if [cart_length for (condition, cart_length) in zip(mask,benchmark_data['cartesian_trajectory_lengths']) if condition == True]:
            mean_cartesian_trajectory_length = statistics.mean([cart_length for (condition, cart_length) in zip(mask,benchmark_data['cartesian_trajectory_lengths']) if condition == True])
            std_dev_cartesian_trajectory_length = statistics.pstdev([cart_length for (condition, cart_length) in zip(mask,benchmark_data['cartesian_trajectory_lengths']) if condition == True])
            max_cartesian_trajectory_length = max([cart_length for (condition, cart_length) in zip(mask,benchmark_data['cartesian_trajectory_lengths']) if condition == True])
            min_cartesian_trajectory_length = min([cart_length for (condition, cart_length) in zip(mask,benchmark_data['cartesian_trajectory_lengths']) if condition == True])
        else:
            mean_cartesian_trajectory_length = -1
            std_dev_cartesian_trajectory_length = -1

        num_disc = [disc for (condition, disc) in zip(success,benchmark_data['discontinuous_joint_trajectory']) if condition == True].count(True)
        ####
        if float(success.count(True)) == 0:
            continous_joint_traj_percent = 0
        else:
            continous_joint_traj_percent = (float(success.count(True)) - float(num_disc))/float(success.count(True))

        joint_trajectory_lengths_without_empty = [joints_length for (condition, joints_length) in zip(success, benchmark_data['joint_trajectory_lengths']) if condition == True]

        #####
        mean_joint_trajectory_lengths = [] 
        std_dev_joint_trajectory_lengths = []

        if joint_trajectory_lengths_without_empty:
            for i in range(len(joint_trajectory_lengths_without_empty[0])):
                if [lengths[i] for lengths in joint_trajectory_lengths_without_empty]:
                    mean_joint_trajectory_lengths.append(statistics.mean([lengths[i] for lengths in joint_trajectory_lengths_without_empty]))
                    std_dev_joint_trajectory_lengths.append(statistics.pstdev([lengths[i] for lengths in joint_trajectory_lengths_without_empty]))
                else:
                    mean_joint_trajectory_lengths.append(-1)
                    std_dev_joint_trajectory_lengths.append(-1)
        else:
            mean_joint_trajectory_lengths = [-1, -1, -1, -1, -1, -1, -1]
            std_dev_joint_trajectory_lengths = [-1, -1, -1, -1, -1, -1, -1]

        if [planning_time for (condition, planning_time) in zip(mask,benchmark_data['planning_times']) if condition == True]:
            mean_planning_time = statistics.mean([planning_time for (condition, planning_time) in zip(mask,benchmark_data['planning_times']) if condition == True])
            std_dev_planning_time = statistics.pstdev([planning_time for (condition, planning_time) in zip(mask,benchmark_data['planning_times']) if condition == True])
            max_planning_time = max([planning_time for (condition, planning_time) in zip(mask,benchmark_data['planning_times']) if condition == True])
            min_planning_time = min([planning_time for (condition, planning_time) in zip(mask,benchmark_data['planning_times']) if condition == True])
        else:
            mean_planning_time = -1
            std_dev_planning_time = -1

        #####
        if [move_time for (condition, move_time) in zip(success,benchmark_data['trajectory_times']) if condition == True]:
            mean_traj_time = statistics.mean([move_time for (condition, move_time) in zip(success,benchmark_data['trajectory_times']) if condition == True])
            std_dev_traj_time = statistics.pstdev([move_time for (condition, move_time) in zip(success,benchmark_data['trajectory_times']) if condition == True])
        else:
            mean_traj_time = -1
            std_dev_traj_time = -1

        to_yaml = {'benchmark_setup': {'planner_id': planner_id, 'waypoint_density': waypoint_density, 'path_constraint': path_constraint, 'object_type': object_type},
        'benchmark_results': {'success_percent': success_percent,
            'mean_cartesian_trajectory_length': mean_cartesian_trajectory_length,
            'std_dev_cartesian_trajectory_length': std_dev_cartesian_trajectory_length,
            'continous_joint_traj_percent': continous_joint_traj_percent,
            'mean_joint_trajectory_lengths': mean_joint_trajectory_lengths,
            'std_dev_joint_trajectory_lengths': std_dev_joint_trajectory_lengths,
            'mean_traj_time': mean_traj_time,
            'std_dev_traj_time': std_dev_traj_time,
            'mean_planning_time': mean_planning_time,
            'std_dev_planning_time': std_dev_planning_time}}

        to_yaml_list.append(to_yaml)

        if waypoint_density == 'sparse' and path_constraint == 'unconstrained' and object_type == 'BOX':
            center = np.array([mean_cartesian_trajectory_length])
            spread = np.array([mean_cartesian_trajectory_length-std_dev_cartesian_trajectory_length,mean_cartesian_trajectory_length+std_dev_cartesian_trajectory_length])
            flier_high = np.array([max_cartesian_trajectory_length])
            flier_low = np.array([min_cartesian_trajectory_length])
            data = np.concatenate((spread, center, flier_high, flier_low))
            data_to_plot['trajectory_length'].append(data)

            center = np.array([mean_planning_time])
            spread = np.array([mean_planning_time-std_dev_planning_time,mean_planning_time+std_dev_planning_time])
            flier_high = np.array([max_planning_time])
            flier_low = np.array([min_planning_time])
            data = np.concatenate((spread, center, flier_high, flier_low))
            data_to_plot['planning_time'].append(data)
            labels_for_plot.append(planner_id)

with open('./benchmark_results.yaml', 'w') as file:
    documents = yaml.dump(to_yaml_list, file, default_flow_style=False)

# fig1, ax = plt.subplots(2)
# ax[0].set_title('Cartesian Trajectory Length')
# # ax[0].set_xlabel('Unconstrained path with sparse waypoints for cylinder object')
# ax[0].set_ylabel('Trajectory length [m]')
# ax[0].boxplot([data_to_plot['trajectory_length'][0],data_to_plot['trajectory_length'][1]], labels = labels_for_plot)

# ax[1].set_title('Planning Time')
# # ax[1].set_xlabel('Unconstrained path with sparse waypoints for cylinder object')
# ax[1].set_ylabel('Planning time [s]')
# ax[1].boxplot([data_to_plot['planning_time'][0],data_to_plot['planning_time'][1]], labels = labels_for_plot)

# fig1.tight_layout()

# plt.show()

# print(waypoint_density, path_constraint, object_type)