% Test Script: test_MultiRobotSim.m

% 清除环境
clear;
clc;
close all;

% 启动计时器
tic;

% 定义多个机器人的初始姿态 [x, y, theta]
initial_poses = [
    0.05, -0.525, 0; % Robot1 起始位置和方向
    0.45, -0.525, 0  % Robot2 在x方向偏移0.40
    -0.45, 0.525, 0  % Robot3 在x方向偏移0.80
    -0.05, 0.525, 0  % Robot4 在x方向偏移1.20
    
];

% 车辆尺寸
robot_length = 0.27;  % 机器人长度（米）
robot_width = 0.19;   % 机器人宽度（米）

% UWB基站位置
uwb_stations = [
    1.35, 1.35; 
    1.35, -1.35;
    -1.35, 1.35;
    -1.35, -1.35
];  % 4个UWB基站的坐标

% 车载UWB偏移
uwb_offset = 0.05;  % 偏移量（米）

% 定义 IMU 偏置
imu_accel_bias = [0.1; -0.05]; % 加速度偏置 (m/s^2)
imu_gyro_bias = 0.02;          % 角速度偏置 (rad/s)

% 创建多机器人仿真管理对象
multiSim = MultiRobotSim(initial_poses, robot_length, robot_width, uwb_stations, uwb_offset, imu_accel_bias, imu_gyro_bias);

% 绘制环境和所有机器人
multiSim.plot_environment();

% 定义基础期望速度
v_d = 0.3; % 基础期望线速度 (m/s)

% % 生成速度参考
% % 定义一个简单的速度参考生成函数
% function v_ref = get_ref_velo(v_d, sim_dt, offset)
%     % 简单的速度参考生成函数
%     % v_d: 基础期望速度
%     % sim_dt: 仿真步长
%     % offset: 偏移量（用于生成不同的速度参考）
% 
%     N = 1000; % 假设最大步数
%     v_ref = v_d * ones(1, N); % 恒定速度
%     % 您可以在这里添加不同的速度变化模式，例如加速、减速等
% end

% 获取机器人数量
num_robots = length(multiSim.robots);

% 为每个机器人生成速度参考
v_ref_all = cell(num_robots, 1);
for i = 1:num_robots
    v_ref_all{i} = get_ref_velo(v_d, multiSim.robots{i}.sim_dt, (i-1)*0.35 + 0.05); % 为每个机器人生成不同的速度参考
end

% 时间参数
dt = multiSim.robots{1}.sim_dt;                % 主循环时间步长 (秒) 对应仿真步长
T = 60;                                        % 总仿真时间 (秒)
t_sim = 0:dt:T;                                % 时间向量
N_sim = length(t_sim);                         % 步数

% 初始化记录变量
% 使用Cell数组来存储每个机器人的数据
time_log = cell(num_robots, 1);
v_log = cell(num_robots, 1);
omega_log = cell(num_robots, 1);
a_linear_log = cell(num_robots, 1);

% 初始化真实位置记录 (2 x N_sim)￥
ground_truth_log = cell(num_robots, 1);
for i = 1:num_robots
    ground_truth_log{i} = zeros(2, N_sim);
end

for i = 1:num_robots
    time_log{i} = zeros(1, N_sim);
    v_log{i} = zeros(1, N_sim);
    omega_log{i} = zeros(1, N_sim);
    a_linear_log{i} = zeros(1, N_sim);
end

% 初始化前一时刻的线加速度为0
last_a_linear = zeros(num_robots, 1);

% 定义绘图更新频率（例如，每隔50步更新一次绘图）
plot_update_interval = multiSim.robots{1}.plot_update_interval;

% 执行仿真，逐步更新机器人的位置并更新图形
for i = 1:N_sim
    current_time = i * dt;             % 当前时间

    % 设置期望速度
    for j = 1:num_robots
        if i <= length(v_ref_all{j})
            desired_v = v_ref_all{j}(i);
        else
            desired_v = v_ref_all{j}(end);
        end
        if desired_v==0
            desired_v=0.0001;
        end
        multiSim.robots{j}.set_desired_velocity(desired_v);
    end

    % 执行一个模拟步长
    multiSim.step_simulation();

    % 记录数据
    for j = 1:num_robots
        v = multiSim.robots{j}.prev_v;
        omega = multiSim.robots{j}.prev_omega;

        % 检查 imu_data.Accel 是否为空
        if ~isempty(multiSim.robots{j}.imu_data.Accel)
            a_linear = multiSim.robots{j}.imu_data.Accel(1,end);
            last_a_linear(j) = a_linear; % 更新最后一个有效的加速度值
        else
            a_linear = last_a_linear(j); % 使用之前存储的值
        end

        v_log{j}(i) = v;
        omega_log{j}(i) = omega;
        time_log{j}(i) = current_time;
        a_linear_log{j}(i) = a_linear;
        ground_truth_log{j}(:, i) = multiSim.robots{j}.pose(1:2); % 记录真实位置￥
    end

    % 每隔一定步数更新绘图
    if mod(i, plot_update_interval) == 0 || i == N_sim
        drawnow limitrate;
    end

    % 检查是否到达最后一个Waypoint for all robots
    all_reached = true;
    for j = 1:num_robots
        final_waypoint = multiSim.robots{j}.pure_pursuit_controller.Waypoints(end, :);
        if norm(multiSim.robots{j}.pose(1:2) - final_waypoint) >= 0.01
            all_reached = false;
            break;
        end
    end
    if all_reached
        disp('所有机器人均已到达最终的Waypoint.');
        % 调整记录变量的长度
        for j = 1:num_robots
            time_log{j} = time_log{j}(1:i);
            v_log{j} = v_log{j}(1:i);
            omega_log{j} = omega_log{j}(1:i);
            a_linear_log{j} = a_linear_log{j}(1:i);
            ground_truth_log{j} = ground_truth_log{j}(:, 1:i); % 裁剪真实位置记录￥
        end
        break;
    end
end

% 保存测量数据和控制记录
multiSim.save_data('measurement_data_multi_robot.mat');
% 保存真实位置数据￥
save('measurement_data_multi_robot.mat', 'ground_truth_log', '-append');

% 可视化记录的数据
figure;

for j = 1:num_robots
    % 绘制线速度
    subplot(num_robots*2, 2, (j-1)*4 + 1);
    plot(time_log{j}, v_log{j}, 'LineWidth', 1.5);
    xlabel('时间 (秒)');
    ylabel(['线速度 v', num2str(j), ' (m/s)']);
    title(['机器人', num2str(j), ' 线速度随时间变化']);
    grid on;

    % 绘制角速度
    subplot(num_robots*2, 2, (j-1)*4 + 2);
    plot(time_log{j}, omega_log{j}, 'LineWidth', 1.5);
    xlabel('时间 (秒)');
    ylabel(['角速度 \omega', num2str(j), ' (rad/s)']);
    title(['机器人', num2str(j), ' 角速度随时间变化']);
    grid on;

    % 绘制UWB距离
    subplot(num_robots*2, 2, (j-1)*4 + 3);
    hold on;
    for k = 1:size(multiSim.robots{j}.uwb_data.Ranges_Tag1,1)
        plot(multiSim.robots{j}.uwb_data.Time, multiSim.robots{j}.uwb_data.Ranges_Tag1(k,:), 'DisplayName', sprintf('基站 %d', k));
    end
    xlabel('时间 (秒)');
    ylabel('距离 (米)');
    title(['机器人', num2str(j), ' UWB 距离随时间变化']);
    legend('show', 'Location', 'best');
    grid on;
    hold off;

    % 绘制线加速度
    subplot(num_robots*2, 2, (j-1)*4 + 4);
    plot(time_log{j}, a_linear_log{j}, 'LineWidth', 1.5);
    xlabel('时间 (秒)');
    ylabel(['线加速度 a', num2str(j), ' (m/s^2)']);
    title(['机器人', num2str(j), ' 线加速度随时间变化']);
    grid on;
end

% 添加相对UWB测量数据的绘图
figure('Name', 'Relative UWB Measurements', 'NumberTitle', 'off');
hold on;

% 获取相对UWB数据
relative_data = multiSim.relative_uwb_data;

% 获取机器人对名称
robot_pairs = fieldnames(relative_data);

% 为每对机器人分配不同的颜色
num_pairs = length(robot_pairs);
pair_colors = lines(num_pairs);

% 定义不同标签对的线型
line_styles = {'-', '--', ':', '-.'};
num_styles = length(line_styles);

% 定义标签对名称
labels = {'Tag1\_Tag1', 'Tag1\_Tag2', 'Tag2\_Tag1', 'Tag2\_Tag2'};

for p = 1:num_pairs
    pair_name = robot_pairs{p};
    data = relative_data.(pair_name);

    % 逐个标签对绘制距离曲线
    for d = 1:size(data.Distances, 1)
        % 选择对应的线型
        style_idx = mod(d-1, num_styles) + 1;
        line_style = line_styles{style_idx};

        % 绘制距离曲线
        plot(data.Time, data.Distances(d, :), 'LineWidth', 1.5, ...
             'Color', pair_colors(p,:), 'LineStyle', line_style, ...
             'DisplayName', [pair_name, ' ', labels{d}]);
    end
end

xlabel('时间 (秒)');
ylabel('距离 (米)');
title('机器人间相对UWB距离测量');
legend('show', 'Location', 'bestoutside');
grid on;
hold off;

% 停止计时器并显示总时间
disp('仿真总时间:');
toc;















% % Test Script: test_WheelRobotSim.m
% % 
% % % 清除环境
% % clear;
% % clc;
% % close all;
% % 
% % % 启动计时器
% % tic;
% % 
% % % 定义初始姿态 [x, y, theta] for Robot1
% % initial_pose1 = [0.1, -0.525, 0];  % 起始位置 (0.1, -0.525) ，方向 0 rad
% % 
% % % 定义初始姿态 [x, y, theta] for Robot2 with a spatial offset of 0.15m
% % offset = 0.15; % 米
% % initial_pose2 = [initial_pose1(1) + offset, initial_pose1(2), initial_pose1(3)]; % 在x方向偏移0.15m
% % 
% % % 车辆尺寸
% % robot_length = 0.27;  % 机器人长度（米）
% % robot_width = 0.19;   % 机器人宽度（米）
% % 
% % % UWB基站位置
% % uwb_stations = [1.35, 1.35; 
% %                 1.35, -1.35;
% %                 -1.35, 1.35;
% %                 -1.35, -1.35];  % 4个UWB基站的坐标
% % 
% % % 车载UWB偏移
% % uwb_offset = 0.1;  % 偏移量（米）
% % 
% % % 定义 IMU 偏置
% % imu_accel_bias = [0.1; -0.05]; % 加速度偏置 (m/s^2)
% % imu_gyro_bias = 0.02;          % 角速度偏置 (rad/s)
% % 
% % % 生成类实例
% % robot1 = WheelRobotSim(initial_pose1, robot_length, robot_width, uwb_stations, uwb_offset, imu_accel_bias, imu_gyro_bias);
% % robot2 = WheelRobotSim(initial_pose2, robot_length, robot_width, uwb_stations, uwb_offset, imu_accel_bias, imu_gyro_bias);
% % 
% % % 绘制初始道路和机器人
% % figure('Name', 'Robot Simulation', 'NumberTitle', 'off');
% % hold on;
% % 
% % % 绘制环境 for both robots
% % robot1.plot_environment();
% % robot2.plot_environment();
% % 
% % % 定义基础期望速度
% % v_d = 0.3; % 基础期望线速度 (m/s)
% % 
% % % 生成速度参考
% % v_ref1 = get_ref_velo(v_d, robot1.sim_dt, 0);          % Robot1 无偏移
% % v_ref2 = get_ref_velo(v_d, robot2.sim_dt, 0.15);       % Robot2 偏移0.15m
% % 
% % % 时间参数
% % dt = robot1.sim_dt;                             % 主循环时间步长 (秒) 对应仿真步长
% % T = 60;                                        % 总仿真时间 (秒)
% % t_sim = 0:dt:T;                                % 时间向量
% % N_sim = length(t_sim);                         % 步数
% % 
% % % 初始化记录变量 for Robot1
% % time_log1 = zeros(1, N_sim);                    % 记录时间
% % v_log1 = zeros(1, N_sim);                       % 记录速度
% % omega_log1 = zeros(1, N_sim);                   % 记录角速度
% % a_linear_log1 = zeros(1, N_sim);                % 记录线加速度
% % 
% % % 初始化记录变量 for Robot2
% % time_log2 = zeros(1, N_sim);                    % 记录时间
% % v_log2 = zeros(1, N_sim);                       % 记录速度
% % omega_log2 = zeros(1, N_sim);                   % 记录角速度
% % a_linear_log2 = zeros(1, N_sim);                % 记录线加速度
% % 
% % % 定义绘图更新频率（例如，每隔50步更新一次绘图）
% % plot_update_interval = robot1.plot_update_interval;
% % 
% % % 执行仿真，逐步更新机器人的位置并更新图形
% % for i = 1:N_sim
% %     current_time = i * dt;             % 当前时间
% % 
% %     % 获取当前速度参考 for Robot1
% %     if i <= length(v_ref1)
% %         desired_v1 = v_ref1(i);
% %     else
% %         desired_v1 = v_ref1(end);
% %     end
% % 
% %     % 获取当前速度参考 for Robot2
% %     if i <= length(v_ref2)
% %         desired_v2 = v_ref2(i);
% %     else
% %         desired_v2 = v_ref2(end);
% %     end
% % 
% %     % 设置期望速度
% %     robot1.set_desired_velocity(desired_v1);
% %     robot2.set_desired_velocity(desired_v2);
% % 
% %     % 执行一个模拟步长，并获取应用的速度指令 for Robot1
% %     [v1, omega1] = robot1.step_simulation();
% % 
% %     % 执行一个模拟步长，并获取应用的速度指令 for Robot2
% %     [v2, omega2] = robot2.step_simulation();
% % 
% %     % 记录速度和控制量 for Robot1
% %     v_log1(i) = v1;
% %     omega_log1(i) = omega1;
% %     time_log1(i) = current_time;
% % 
% %     % 记录线加速度 for Robot1
% %     if i == 1
% %         a_linear_log1(i) = robot1.imu_data.Accel(1,end); % 初始加速度
% %     else
% %         a_linear_log1(i) = robot1.imu_data.Accel(1,end); % 从IMU获取线加速度
% %     end
% % 
% %     % 记录速度 and control for Robot2
% %     v_log2(i) = v2;
% %     omega_log2(i) = omega2;
% %     time_log2(i) = current_time;
% % 
% %     % 记录线加速度 for Robot2
% %     if i == 1
% %         a_linear_log2(i) = robot2.imu_data.Accel(1,end); % 初始加速度
% %     else
% %         a_linear_log2(i) = robot2.imu_data.Accel(1,end); % 从IMU获取线加速度
% %     end
% % 
% %     % 每隔一定步数更新绘图
% %     if mod(i, plot_update_interval) == 0 || i == N_sim
% %         % 由于绘图已经在类内部优化，这里不需要额外处理
% %         drawnow limitrate;
% %     end
% % 
% %     % 检查是否到达最后一个Waypoint for both robots
% %     if norm(robot1.pose(1:2) - robot1.centerline1(end, :)) < 0.01 && ...
% %        norm(robot2.pose(1:2) - robot2.centerline1(end, :)) < 0.01
% %         disp('两辆机器人均已到达最终的Waypoint.');
% %         % 调整记录变量的长度
% %         time_log1 = time_log1(1:i);
% %         v_log1 = v_log1(1:i);
% %         omega_log1 = omega_log1(1:i);
% %         a_linear_log1 = a_linear_log1(1:i);
% % 
% %         time_log2 = time_log2(1:i);
% %         v_log2 = v_log2(1:i);
% %         omega_log2 = omega_log2(1:i);
% %         a_linear_log2 = a_linear_log2(1:i);
% %         break;
% %     end
% % end
% % 
% % % 保存测量数据和控制记录
% % robot1.save_data('measurement_data_robot1.mat');
% % robot2.save_data('measurement_data_robot2.mat');
% % 
% % % 可视化记录的数据
% % figure;
% % 
% % % 绘制线速度 for Robot1
% % subplot(4,2,1);
% % plot(time_log1, v_log1, 'b-', 'LineWidth', 1.5);
% % xlabel('时间 (秒)');
% % ylabel('线速度 v1 (m/s)');
% % title('机器人1线速度随时间变化');
% % grid on;
% % 
% % % 绘制线速度 for Robot2
% % subplot(4,2,2);
% % plot(time_log2, v_log2, 'c-', 'LineWidth', 1.5);
% % xlabel('时间 (秒)');
% % ylabel('线速度 v2 (m/s)');
% % title('机器人2线速度随时间变化');
% % grid on;
% % 
% % % 绘制角速度 for Robot1
% % subplot(4,2,3);
% % plot(time_log1, omega_log1, 'r-', 'LineWidth', 1.5);
% % xlabel('时间 (秒)');
% % ylabel('角速度 \omega1 (rad/s)');
% % title('机器人1角速度随时间变化');
% % grid on;
% % 
% % % 绘制角速度 for Robot2
% % subplot(4,2,4);
% % plot(time_log2, omega_log2, 'm-', 'LineWidth', 1.5);
% % xlabel('时间 (秒)');
% % ylabel('角速度 \omega2 (rad/s)');
% % title('机器人2角速度随时间变化');
% % grid on;
% % 
% % % 绘制UWB距离 for Robot1
% % subplot(4,2,5);
% % hold on;
% % for k = 1:size(robot1.uwb_data.Ranges_Tag1,1)
% %     plot(robot1.uwb_data.Time, robot1.uwb_data.Ranges_Tag1(k,:), 'DisplayName', sprintf('基站 %d', k));
% % end
% % xlabel('时间 (秒)');
% % ylabel('距离 (米)');
% % title('机器人1 UWB 距离随时间变化');
% % legend('show');
% % grid on;
% % hold off;
% % 
% % % 绘制UWB距离 for Robot2
% % subplot(4,2,6);
% % hold on;
% % for k = 1:size(robot2.uwb_data.Ranges_Tag1,1)
% %     plot(robot2.uwb_data.Time, robot2.uwb_data.Ranges_Tag1(k,:), 'DisplayName', sprintf('基站 %d', k));
% % end
% % xlabel('时间 (秒)');
% % ylabel('距离 (米)');
% % title('机器人2 UWB 距离随时间变化');
% % legend('show');
% % grid on;
% % hold off;
% % 
% % % 绘制线加速度 for Robot1
% % subplot(4,2,7);
% % plot(time_log1, a_linear_log1, 'g-', 'LineWidth', 1.5);
% % xlabel('时间 (秒)');
% % ylabel('线加速度 a1 (m/s^2)');
% % title('机器人1线加速度随时间变化');
% % grid on;
% % 
% % % 绘制线加速度 for Robot2
% % subplot(4,2,8);
% % plot(time_log2, a_linear_log2, 'k-', 'LineWidth', 1.5);
% % xlabel('时间 (秒)');
% % ylabel('线加速度 a2 (m/s^2)');
% % title('机器人2线加速度随时间变化');
% % grid on;
% % 
% % % 停止计时器并显示总时间
% % disp('仿真总时间:');
% % toc;