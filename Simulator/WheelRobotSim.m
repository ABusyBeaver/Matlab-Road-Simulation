classdef WheelRobotSim < handle
    properties
        % 基本属性
        pose                    % Robot position [x, y, theta]
        robot_length            % Robot length
        robot_width             % Robot width
        uwb_stations            % UWB base station positions
        uwb_offset              % Car-mounted UWB offset

        % 传感器数据存储
        imu_data                % IMU 数据存储结构
        uwb_data                % UWB 数据存储结构
        pose_data               % Pose 数据存储结构
        center_distances_data   % 机器人中心到基站的距离数据存储结构  % <--- New Property

        % 图形句柄
        traj_line_plot
        robot_patch_plot
        dir_line_plot
        car_uwb_marker_plot

        % 模拟参数
        sim_frequency = 1000;    % Simulation update frequency in Hz
        imu_frequency = 100;     % IMU sampling frequency in Hz
        uwb_frequency = 20;      % UWB sampling frequency in Hz

        sim_dt                  % Simulation time step
        imu_dt                  % IMU sampling time step
        uwb_dt                  % UWB sampling time step
        next_imu_time           % Next IMU sample time
        next_uwb_time           % Next UWB sample time
        simulation_duration = 100; % Total simulation duration in seconds

        % IMU 偏置
        imu_accel_bias = [0; 0]; % IMU 加速度偏置 [ax_bias; ay_bias] (m/s^2)
        imu_gyro_bias = 0;        % IMU 角速度偏置 (rad/s)

        % 内部时间跟踪
        internal_time = 0;        % 内部模拟时间 (秒)

        % 控制器
        pure_pursuit_controller  % controllerPurePursuit 对象

        % 绘图更新参数
        plot_update_interval = 50; % 每隔多少步更新一次绘图

        % 前一时刻的速度和角速度
        prev_v = 0;               % 前一时刻的线速度 (m/s)
        prev_omega = 0;           % 前一时刻的角速度 (rad/s)
    end

    methods
        function obj = WheelRobotSim(initial_pose, robot_length, robot_width, uwb_stations, uwb_offset, imu_accel_bias, imu_gyro_bias)
            % 构造函数
            obj.pose = initial_pose;
            obj.robot_length = robot_length;
            obj.robot_width = robot_width;
            obj.uwb_stations = uwb_stations;
            obj.uwb_offset = uwb_offset;  % 设置车载 UWB 偏移

            % 初始化传感器数据存储结构
            obj.imu_data = struct('Time', [], 'Accel', [], 'Gyro', []);
            obj.uwb_data = struct('Time', [], 'Ranges_Tag1', [], 'Ranges_Tag2', []);
            obj.pose_data = struct('Time', [], 'X', [], 'Y', [], 'Yaw', []);
            obj.center_distances_data = struct('Time', [], 'Distances', []); % <--- Initialize Center Distances Data

            % 计算时间步长
            obj.sim_dt = 1 / obj.sim_frequency;
            obj.imu_dt = 1 / obj.imu_frequency;
            obj.uwb_dt = 1 / obj.uwb_frequency;

            % 设置 IMU 偏置（如果提供）
            if nargin >= 6 && ~isempty(imu_accel_bias)
                obj.imu_accel_bias = imu_accel_bias;
            end
            if nargin >= 7 && ~isempty(imu_gyro_bias)
                obj.imu_gyro_bias = imu_gyro_bias;
            end

            % 初始化下一个采样时间
            obj.next_imu_time = obj.imu_dt;
            obj.next_uwb_time = obj.uwb_dt;

            % 初始化控制器
            obj.initialize_controller();
        end

        function initialize_controller(obj)
            % 初始化 controllerPurePursuit 控制器
            obj.pure_pursuit_controller = controllerPurePursuit('Waypoints', [0 0], ...
                                                                 'DesiredLinearVelocity', 0.01, ...
                                                                 'MaxAngularVelocity', pi);
            % 设置 Lookahead Distance（可选，根据需要调整）
            obj.pure_pursuit_controller.LookaheadDistance = 0.1;
        end

        function set_waypoints(obj, waypoints)
            % 设置控制器的路点
            obj.pure_pursuit_controller.Waypoints = waypoints;
        end

        function set_desired_velocity(obj, desired_v)
            % 设置控制器的期望线速度
            desired_v = abs(desired_v); % 确保非负
            obj.pure_pursuit_controller.DesiredLinearVelocity = desired_v;
        end

        function initialize_plot(obj, ax, color)
            % 初始化机器人在指定轴上的绘图
            hold(ax, 'on');
            if nargin < 3
                color = 'b'; % 默认颜色
            end

            % 绘制轨迹线
            obj.traj_line_plot = plot(ax, obj.pose(1), obj.pose(2), '-', 'Color', color, 'LineWidth', 1, 'DisplayName', '轨迹');

            % 绘制机器人矩形
            [robot_X, robot_Y] = obj.transformRectangle();
            obj.robot_patch_plot = plot(ax, [robot_X; robot_X(1)], [robot_Y; robot_Y(1)], '-', 'Color', color, 'LineWidth', 2, 'DisplayName', '机器人');

            % 绘制方向线
            obj.dir_line_plot = plot(ax, [obj.pose(1), obj.pose(1) + obj.robot_length * cos(obj.pose(3))], ...
                                     [obj.pose(2), obj.pose(2) + obj.robot_length * sin(obj.pose(3))], ...
                                     '-', 'Color', color, 'LineWidth', 2, 'DisplayName', '方向');

            % 绘制车载UWB标记（前后）
            obj.car_uwb_marker_plot(1) = plot(ax, obj.pose(1) + obj.uwb_offset * cos(obj.pose(3)), ...
                                             obj.pose(2) + obj.uwb_offset * sin(obj.pose(3)), ...
                                             's', 'MarkerSize', 8, 'MarkerFaceColor', color, 'DisplayName', '车载UWB 前'); % 前车载UWB

            obj.car_uwb_marker_plot(2) = plot(ax, obj.pose(1) - obj.uwb_offset * cos(obj.pose(3)), ...
                                             obj.pose(2) - obj.uwb_offset * sin(obj.pose(3)), ...
                                             's', 'MarkerSize', 8, 'MarkerFaceColor', color, 'DisplayName', '车载UWB 后'); % 后车载UWB
        end

        function [X, Y] = transformRectangle(obj)
            % 将机器人矩形从局部坐标系转换到全局坐标系
            half_length = obj.robot_length / 2;
            half_width = obj.robot_width / 2;

            % 机器人矩形在局部坐标系下的四个角点
            local_coords = [-half_length, -half_width;
                            half_length, -half_width;
                            half_length, half_width;
                            -half_length, half_width];

            % 旋转矩阵
            R = [cos(obj.pose(3)), -sin(obj.pose(3)); 
                 sin(obj.pose(3)), cos(obj.pose(3))];

            % 将局部坐标转换为全局坐标
            global_coords = (R * local_coords')' + obj.pose(1:2);

            % 提取全局坐标
            X = global_coords(:, 1);
            Y = global_coords(:, 2);
        end

        function [v, omega] = step_simulation(obj)
            % 执行一个模拟步长，并在必要时采样传感器数据
            % 返回当前应用的线速度和角速度

            % 获取控制器的控制指令（线速度和角速度）
            [v, omega] = obj.pure_pursuit_controller(obj.pose);

            % 计算加速度
            a_linear = (v - obj.prev_v) / obj.sim_dt;    % 线加速度 (m/s^2)
            a_angular = (omega - obj.prev_omega) / obj.sim_dt; % 角加速度 (rad/s^2)

            % 应用控制输入并更新姿态
            obj.apply_control(v, omega, obj.sim_dt);
            obj.internal_time = obj.internal_time + obj.sim_dt;

            % 更新轨迹记录
            obj.traj_line_plot.XData = [obj.traj_line_plot.XData, obj.pose(1)];
            obj.traj_line_plot.YData = [obj.traj_line_plot.YData, obj.pose(2)];

            % 更新机器人图形
            [robot_X, robot_Y] = obj.transformRectangle();
            if ishandle(obj.robot_patch_plot)
                set(obj.robot_patch_plot, 'XData', [robot_X; robot_X(1)], 'YData', [robot_Y; robot_Y(1)]);
            else
                % Handle invalid plot handle
                obj.robot_patch_plot = plot(obj.traj_line_plot.Parent, [robot_X; robot_X(1)], [robot_Y; robot_Y(1)], ...
                    '-', 'Color', obj.traj_line_plot.Color, 'LineWidth', 2, 'DisplayName', '机器人');
            end

            % 更新方向线
            if ishandle(obj.dir_line_plot)
                set(obj.dir_line_plot, 'XData', [obj.pose(1), obj.pose(1) + obj.robot_length * cos(obj.pose(3))], ...
                                      'YData', [obj.pose(2), obj.pose(2) + obj.robot_length * sin(obj.pose(3))]);
            else
                obj.dir_line_plot = plot(obj.traj_line_plot.Parent, [obj.pose(1), obj.pose(1) + obj.robot_length * cos(obj.pose(3))], ...
                                         [obj.pose(2), obj.pose(2) + obj.robot_length * sin(obj.pose(3))], ...
                                         '-', 'Color', obj.traj_line_plot.Color, 'LineWidth', 2, 'DisplayName', '方向');
            end

            % 更新车载UWB标记（前后）
            if ishandle(obj.car_uwb_marker_plot(1))
                set(obj.car_uwb_marker_plot(1), 'XData', obj.pose(1) + obj.uwb_offset * cos(obj.pose(3)), ...
                                               'YData', obj.pose(2) + obj.uwb_offset * sin(obj.pose(3)));
            else
                obj.car_uwb_marker_plot(1) = plot(obj.traj_line_plot.Parent, obj.pose(1) + obj.uwb_offset * cos(obj.pose(3)), ...
                                                 obj.pose(2) + obj.uwb_offset * sin(obj.pose(3)), ...
                                                 's', 'MarkerSize', 8, 'MarkerFaceColor', obj.traj_line_plot.Color, 'DisplayName', '车载UWB 前'); % 前车载UWB
            end

            if ishandle(obj.car_uwb_marker_plot(2))
                set(obj.car_uwb_marker_plot(2), 'XData', obj.pose(1) - obj.uwb_offset * cos(obj.pose(3)), ...
                                               'YData', obj.pose(2) - obj.uwb_offset * sin(obj.pose(3)));
            else
                obj.car_uwb_marker_plot(2) = plot(obj.traj_line_plot.Parent, obj.pose(1) - obj.uwb_offset * cos(obj.pose(3)), ...
                                                 obj.pose(2) - obj.uwb_offset * sin(obj.pose(3)), ...
                                                 's', 'MarkerSize', 8, 'MarkerFaceColor', obj.traj_line_plot.Color, 'DisplayName', '车载UWB 后'); % 后车载UWB
            end

            % 检查是否需要采样 IMU
            if obj.internal_time >= obj.next_imu_time
                obj.simulate_imu(obj.next_imu_time, a_linear, a_angular); % 使用下一个采样时间
                obj.next_imu_time = obj.next_imu_time + obj.imu_dt;
            end

            % 检查是否需要采样 UWB 和 Pose
            if obj.internal_time >= obj.next_uwb_time
                obj.simulate_uwb(obj.next_uwb_time); % 使用当前时间
                obj.simulate_pose(obj.next_uwb_time); % 记录Pose数据
                obj.simulate_center_distances(obj.next_uwb_time); % <--- Simulate Center Distances
                obj.next_uwb_time = obj.next_uwb_time + obj.uwb_dt;
            end

            % 更新前一时刻的速度和角速度
            obj.prev_v = v;
            obj.prev_omega = omega;
        end

        function simulate_imu(obj, current_time, a_linear, a_angular)
            % 模拟 IMU 测量数据（加速度和角速度）
            % current_time: 当前仿真时间（秒）
            % a_linear: 当前线加速度 (m/s^2)
            % a_angular: 当前角加速度 (rad/s^2)

            % 计算IMU在机器人局部坐标系下的加速度
            % a_local_x = a_linear
            % a_local_y = v * omega (向心加速度)
            a_local_x = a_linear;
            a_local_y = obj.prev_v * obj.prev_omega;

            % IMU 加速度测量（在机器人局部坐标系下）
            imu_accel = [a_local_x; a_local_y] + obj.imu_accel_bias;

            % IMU 角速度测量
            imu_gyro = obj.prev_omega + obj.imu_gyro_bias;

            % 添加噪声（根据实际传感器特性调整噪声参数）
            accel_noise_std = 0.02; % m/s^2
            gyro_noise_std = 0.01;  % rad/s

            imu_accel_noisy = imu_accel + accel_noise_std * randn(2,1);
            imu_gyro_noisy = imu_gyro + gyro_noise_std * randn;

            % 记录数据
            obj.imu_data.Time(end+1) = current_time;
            obj.imu_data.Accel(:,end+1) = imu_accel_noisy;
            obj.imu_data.Gyro(end+1) = imu_gyro_noisy;
        end

        function simulate_uwb(obj, current_time)
            % 模拟 UWB 测量数据（到各基站的距离）
            % current_time: 当前仿真时间（秒）

            % 计算车载的两个uwb tag到每个uwb anchor的距离
            tag1_pos = [obj.pose(1) + obj.uwb_offset * cos(obj.pose(3));
                        obj.pose(2) + obj.uwb_offset * sin(obj.pose(3))];
            tag2_pos = [obj.pose(1) - obj.uwb_offset * cos(obj.pose(3));
                        obj.pose(2) - obj.uwb_offset * sin(obj.pose(3))];

            num_stations = size(obj.uwb_stations, 1);
            distances_tag1 = zeros(num_stations, 1);
            distances_tag2 = zeros(num_stations, 1);

            for i = 1:num_stations
                distances_tag1(i) = norm(obj.uwb_stations(i, :)' - tag1_pos);
                distances_tag2(i) = norm(obj.uwb_stations(i, :)' - tag2_pos);
            end

            % 添加噪声（根据实际传感器特性调整噪声参数）
            range_noise_std = 0.05; % 米
            distances_tag1_noisy = distances_tag1 + range_noise_std * randn(num_stations, 1);
            distances_tag2_noisy = distances_tag2 + range_noise_std * randn(num_stations, 1);

            % 存储数据
            obj.uwb_data.Time(end+1) = current_time;
            obj.uwb_data.Ranges_Tag1(:, end+1) = distances_tag1_noisy;
            obj.uwb_data.Ranges_Tag2(:, end+1) = distances_tag2_noisy;
        end

        function simulate_center_distances(obj, current_time)
            % 模拟机器人中心到各基站的距离测量
            % current_time: 当前仿真时间（秒）

            % 计算机器人中心到每个基站的距离
            robot_center = obj.pose(1:2);
            num_stations = size(obj.uwb_stations, 1);
            distances = zeros(num_stations, 1);
            for i = 1:num_stations
                distances(i) = norm(obj.uwb_stations(i, :)' - robot_center);
            end

            % 添加噪声（与 UWB 保持一致）
            range_noise_std = 0.05; % 米
            distances_noisy = distances + range_noise_std * randn(num_stations, 1);

            % 存储数据
            obj.center_distances_data.Time(end+1) = current_time;
            obj.center_distances_data.Distances(:, end+1) = distances_noisy;
        end

        function simulate_pose(obj, current_time)
            % 记录机器人位姿数据
            % current_time: 当前仿真时间（秒）

            % 提取当前姿态
            x = obj.pose(1);
            y = obj.pose(2);
            yaw = obj.pose(3);

            % 存储数据
            obj.pose_data.Time(end+1) = current_time;
            obj.pose_data.X(end+1) = x;
            obj.pose_data.Y(end+1) = y;
            obj.pose_data.Yaw(end+1) = yaw;
        end

        function apply_control(obj, v, omega, dt)
            % 应用控制输入并更新姿态
            % v: 线速度 (m/s)
            % omega: 角速度 (rad/s)
            % dt: 时间步长 (s)

            % 简单的单轨模型
            obj.pose(1) = obj.pose(1) + v * cos(obj.pose(3)) * dt;
            obj.pose(2) = obj.pose(2) + v * sin(obj.pose(3)) * dt;
            obj.pose(3) = obj.pose(3) + omega * dt;
        end

        function save_data(obj, filename)
            % 保存传感器测量数据到 .mat 文件
            imu_data = obj.imu_data;
            uwb_data = obj.uwb_data;
            pose_data = obj.pose_data;
            center_distances_data = obj.center_distances_data; % <--- Include Center Distances Data
            save(filename, 'imu_data', 'uwb_data', 'pose_data', 'center_distances_data');
        end
    end
end
