% car_1_anchor_data_1  % 车辆1第1个标签到4个基站的距离 [num_tags x num_steps] 例如 2x1200
% car_2_anchor_data_1  % 车辆2第1个标签到4个基站的距离 [num_tags x num_steps]
% car_1_car_2_relative_data  % 车辆1和车辆2的标签之间的相对距离 [num_tags x num_tags x num_steps]
% car_1_imu_data       % 车辆1的IMU数据 [3 x num_imu_steps] 例如 3x30000
% car_2_imu_data       % 车辆2的IMU数据 [3 x num_imu_steps]

car_1_anchor_data_1 = multiSim.robots{1,1}.uwb_data.Ranges_Tag1;
car_1_anchor_data_2 = multiSim.robots{1,1}.uwb_data.Ranges_Tag2;
car_2_anchor_data_1 = multiSim.robots{2,1}.uwb_data.Ranges_Tag1;
car_2_anchor_data_2 = multiSim.robots{2,1}.uwb_data.Ranges_Tag2;
car_3_anchor_data_1 = multiSim.robots{3,1}.uwb_data.Ranges_Tag1;
car_3_anchor_data_2 = multiSim.robots{3,1}.uwb_data.Ranges_Tag2;
car_4_anchor_data_1 = multiSim.robots{4,1}.uwb_data.Ranges_Tag1;
car_4_anchor_data_2 = multiSim.robots{4,1}.uwb_data.Ranges_Tag2;

% 相对距离
car_1_car_2_relative_data = multiSim.relative_uwb_data.robot1_robot2.Distances;
car_1_car_3_relative_data = multiSim.relative_uwb_data.robot1_robot3.Distances;
car_1_car_4_relative_data = multiSim.relative_uwb_data.robot1_robot4.Distances;
car_2_car_3_relative_data = multiSim.relative_uwb_data.robot2_robot3.Distances;
car_2_car_4_relative_data = multiSim.relative_uwb_data.robot2_robot4.Distances;
car_3_car_4_relative_data = multiSim.relative_uwb_data.robot3_robot4.Distances;

% imu数据
car_1_imu_data = [multiSim.robots{1,1}.imu_data.Accel ; multiSim.robots{1,1}.imu_data.Gyro];
car_2_imu_data = [multiSim.robots{2,1}.imu_data.Accel ; multiSim.robots{2,1}.imu_data.Gyro];
car_3_imu_data = [multiSim.robots{3,1}.imu_data.Accel ; multiSim.robots{3,1}.imu_data.Gyro];
car_4_imu_data = [multiSim.robots{4,1}.imu_data.Accel ; multiSim.robots{4,1}.imu_data.Gyro];

% 真值
car_1_true_pose = [multiSim.robots{1,1}.pose_data.X; multiSim.robots{1,1}.pose_data.Y; multiSim.robots{1,1}.pose_data.Yaw];
car_2_true_pose = [multiSim.robots{2,1}.pose_data.X; multiSim.robots{2,1}.pose_data.Y; multiSim.robots{2,1}.pose_data.Yaw];
car_3_true_pose = [multiSim.robots{3,1}.pose_data.X; multiSim.robots{3,1}.pose_data.Y; multiSim.robots{3,1}.pose_data.Yaw];
car_4_true_pose = [multiSim.robots{4,1}.pose_data.X; multiSim.robots{4,1}.pose_data.Y; multiSim.robots{4,1}.pose_data.Yaw];


car_1_center_uwb_data = multiSim.robots{1,1}.center_distances_data.Distances;
car_2_center_uwb_data = multiSim.robots{2,1}.center_distances_data.Distances;
car_3_center_uwb_data = multiSim.robots{3,1}.center_distances_data.Distances;
car_4_center_uwb_data = multiSim.robots{4,1}.center_distances_data.Distances;

save('simulation_test_data.mat', ...
    'car_1_anchor_data_1', 'car_1_anchor_data_2', 'car_2_anchor_data_1', 'car_2_anchor_data_2',...
    'car_3_anchor_data_1', 'car_3_anchor_data_2', 'car_4_anchor_data_1', 'car_4_anchor_data_2',...
    'car_1_car_2_relative_data', 'car_1_car_3_relative_data', 'car_1_car_4_relative_data', 'car_2_car_3_relative_data', 'car_2_car_4_relative_data', 'car_3_car_4_relative_data',...
    'car_1_imu_data', 'car_2_imu_data', 'car_3_imu_data', 'car_4_imu_data',...
    'car_1_true_pose', 'car_2_true_pose','car_3_true_pose','car_4_true_pose', ...
    'car_1_center_uwb_data', 'car_2_center_uwb_data','car_3_center_uwb_data','car_4_center_uwb_data');