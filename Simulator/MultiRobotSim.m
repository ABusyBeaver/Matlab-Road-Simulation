classdef MultiRobotSim < handle
    properties
        robots          % Cell数组，每个单元是一个 WheelRobotSim 实例
        uwb_stations    % UWB基站位置
        uwb_offset      % 车载UWB偏移
        road_polygon    % 道路多边形
        divider         % 分隔线
        centerline1     % Centerline for lane1
        centerline2     % Centerline for lane2
        left_border1    % Left boundary for lane1
        right_border1   % Right boundary for lane1
        left_border2    % Left boundary for lane2
        right_border2   % Right boundary for lane2
        
        % 新增属性：相对UWB测量数据
        relative_uwb_data          % 结构体，存储相对UWB测量数据
        relative_uwb_frequency = 20;  % 相对UWB采样频率（Hz）
        relative_uwb_dt             % 相对UWB采样时间步长
        next_relative_uwb_time      % 下一个相对UWB采样时间
    end
    
    methods
        function obj = MultiRobotSim(initial_poses, robot_length, robot_width, uwb_stations, uwb_offset, imu_accel_bias, imu_gyro_bias)
            % 构造函数，创建多个 WheelRobotSim 实例
            num_robots = size(initial_poses, 1);
            obj.robots = cell(num_robots, 1);
            for i = 1:num_robots
                obj.robots{i} = WheelRobotSim(initial_poses(i, :), robot_length, robot_width, uwb_stations, uwb_offset, imu_accel_bias, imu_gyro_bias);
            end
            obj.uwb_stations = uwb_stations;
            obj.uwb_offset = uwb_offset;
            % 生成道路数据，假设所有机器人共享相同的道路
            obj.generate_road();
            
            % 初始化相对UWB测量数据
            obj.relative_uwb_data = struct();
            for i = 1:num_robots
                for j = i+1:num_robots
                    pair_name = sprintf('robot%d_robot%d', i, j);
                    obj.relative_uwb_data.(pair_name).Time = [];
                    obj.relative_uwb_data.(pair_name).Distances = [];
                end
            end
            
            % 设置相对UWB采样时间步长
            obj.relative_uwb_dt = 1 / obj.relative_uwb_frequency;
            obj.next_relative_uwb_time = obj.relative_uwb_dt;
        end
        
        function generate_road(obj)
            % 生成道路数据
            lane_width = 0.35;                 % 每个车道的宽度（米）
            road_half_width = lane_width / 2;  % 半宽度

            % 定义两个车道的中心线偏移量
            y_offsets_lane1 = [-0.525, 0.525];
            y_offsets_lane2 = [-0.175, 0.175];

            % 生成中心线
            centerline1 = generate_centerline_custom(y_offsets_lane1);
            centerline2 = generate_centerline_custom(y_offsets_lane2);

            obj.centerline1 = centerline1;
            obj.centerline2 = centerline2;

            % 生成边界线
            [left_border1, right_border1] = generate_boundaries(centerline1, road_half_width);
            [left_border2, right_border2] = generate_boundaries(centerline2, road_half_width);

            obj.left_border1 = left_border1;
            obj.right_border1 = right_border1;
            obj.left_border2 = left_border2;
            obj.right_border2 = right_border2;

            % 生成路面多边形
            obj.road_polygon = generate_road_polygon(right_border1, left_border2);

            % 计算分隔线（位于车道1右边界和车道2左边界之间的中点）
            obj.divider = (right_border1 + left_border2) / 2;

            for i = 1:length(obj.robots)
                obj.robots{i}.set_waypoints([obj.centerline1; obj.centerline1]);
            end
        end
        
        function simulate_relative_uwb(obj, current_time)
            % 模拟机器人间的相对UWB测量
            num_robots = length(obj.robots);
            for i = 1:num_robots
                for j = i+1:num_robots
                    % 获取机器人i的两个UWB标签位置
                    tag1_i = obj.robots{i}.pose(1:2) + ([cos(obj.robots{i}.pose(3)), -sin(obj.robots{i}.pose(3)); sin(obj.robots{i}.pose(3)), cos(obj.robots{i}.pose(3))] *  [obj.uwb_offset; 0])';
                    tag2_i = obj.robots{i}.pose(1:2) - ([cos(obj.robots{i}.pose(3)), -sin(obj.robots{i}.pose(3)); sin(obj.robots{i}.pose(3)), cos(obj.robots{i}.pose(3))] *  [obj.uwb_offset; 0])';
                    
                    % 获取机器人j的两个UWB标签位置
                    tag1_j = obj.robots{j}.pose(1:2) + ([cos(obj.robots{j}.pose(3)), -sin(obj.robots{j}.pose(3)); sin(obj.robots{j}.pose(3)), cos(obj.robots{j}.pose(3))] *  [obj.uwb_offset; 0])';
                    tag2_j = obj.robots{j}.pose(1:2) - ([cos(obj.robots{j}.pose(3)), -sin(obj.robots{j}.pose(3)); sin(obj.robots{j}.pose(3)), cos(obj.robots{j}.pose(3))] *  [obj.uwb_offset; 0])';
                    
                    % 计算四个标签对之间的距离
                    distances = [
                        norm(tag1_i - tag1_j);
                        norm(tag1_i - tag2_j);
                        norm(tag2_i - tag1_j);
                        norm(tag2_i - tag2_j)
                    ];
                    
                    % 添加噪声
                    range_noise_std = 0.05; % 米
                    distances_noisy = distances + range_noise_std * randn(size(distances));
                    
                    % 存储数据
                    pair_name = sprintf('robot%d_robot%d', i, j);
                    obj.relative_uwb_data.(pair_name).Time(end+1) = current_time;
                    obj.relative_uwb_data.(pair_name).Distances(:, end+1) = distances_noisy;
                end
            end
        end
        
        function step_simulation(obj)
            % 对所有机器人执行仿真步长
            for i = 1:length(obj.robots)
                obj.robots{i}.step_simulation();
            end
            
            % 获取当前仿真时间（假设所有机器人时间一致）
            current_time = obj.robots{1}.internal_time;
            
            % 检查是否需要采样相对UWB
            if current_time >= obj.next_relative_uwb_time
                obj.simulate_relative_uwb(obj.next_relative_uwb_time); % 使用采样时间
                obj.next_relative_uwb_time = obj.next_relative_uwb_time + obj.relative_uwb_dt;
            end
        end
        
        function plot_environment(obj)
            % 绘制所有机器人的环境
            figure('Name', 'Multi-Robot Simulation', 'NumberTitle', 'off');
            hold on;

            % 绘制路面多边形
            fill(obj.road_polygon(:,1), obj.road_polygon(:,2), [0.6 0.6 0.6], ...
                'EdgeColor', 'none', 'DisplayName', '路面'); % 灰色路面

            % 绘制车道中心线
            plot(obj.centerline1(:,1), obj.centerline1(:,2), ...
                'y--', 'LineWidth', 1.5, 'DisplayName', '中心线1'); % 虚线黄色中心线1
            plot(obj.centerline2(:,1), obj.centerline2(:,2), ...
                'y--', 'LineWidth', 1.5, 'DisplayName', '中心线2'); % 虚线黄色中心线2

            % 绘制车道边界
            plot(obj.right_border1(:,1), obj.right_border1(:,2), ...
                'k-', 'LineWidth', 2, 'DisplayName', '车道1边界'); % 加粗实线
            plot(obj.left_border2(:,1), obj.left_border2(:,2), ...
                'k-', 'LineWidth', 2, 'DisplayName', '车道2边界'); % 加粗实线

            % 绘制分隔线
            plot(obj.divider(:,1), obj.divider(:,2), ...
                'w--', 'LineWidth', 3, 'DisplayName', '分隔线'); % 加粗白色虚线

            % 绘制UWB基站
            for i = 1:size(obj.uwb_stations,1)
                if i == 1
                    plot(obj.uwb_stations(i,1), obj.uwb_stations(i,2), ...
                        'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'UWB基站'); % 红色圆圈
                else
                    plot(obj.uwb_stations(i,1), obj.uwb_stations(i,2), ...
                        'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8); % 无重复DisplayName
                end
            end

            % 分配不同的颜色给机器人
            colors = lines(length(obj.robots));

            % 绘制每个机器人
            for i = 1:length(obj.robots)
                robot = obj.robots{i};
                robot.initialize_plot(gca, colors(i,:));
            end

            % 可选：绘制相对UWB测量
            obj.plot_relative_uwb();

            % 设置图形属性
            xlabel('X (米)');
            ylabel('Y (米)');
            legend('show', 'Location', 'best');
            title('多机器人仿真');
            axis equal;
            grid on;

            hold off;
        end
        
        function plot_relative_uwb(obj)
            % 绘制机器人间的相对UWB测量
            hold on;
            num_robots = length(obj.robots);
            colors = lines(num_robots);
            
            for i = 1:num_robots
                for j = i+1:num_robots
                    % 获取当前机器人间的相对UWB测量
                    pair_name = sprintf('robot%d_robot%d', i, j);
                    if isempty(obj.relative_uwb_data.(pair_name).Distances)
                        continue;
                    end
                    latest_distances = obj.relative_uwb_data.(pair_name).Distances(:, end);
                    
                    % 选择一个代表性的距离（例如第一个标签对的距离）
                    distance = latest_distances(1);
                    
                    % 获取机器人位置
                    pos_i = obj.robots{i}.pose(1:2);
                    pos_j = obj.robots{j}.pose(1:2);
                    
                    % 绘制连线
                    plot([pos_i(1), pos_j(1)], [pos_i(2), pos_j(2)], 'Color', colors(i,:), 'LineStyle', '--', 'DisplayName', sprintf('R%d-R%d Distance', i, j));
                    
                    % 可选：在连线上显示距离值
                    mid_point = (pos_i + pos_j) / 2;
                    text(mid_point(1), mid_point(2), sprintf('%.2f m', distance), 'Color', colors(i,:), 'FontSize', 8, 'HorizontalAlignment', 'center');
                end
            end
            
            hold off;
        end
        
        function save_data(obj, filename)
            % 保存所有机器人的传感器数据到一个 .mat 文件
            data = struct();
            for i = 1:length(obj.robots)
                data.(['robot', num2str(i), '_imu']) = obj.robots{i}.imu_data;
                data.(['robot', num2str(i), '_uwb']) = obj.robots{i}.uwb_data;
            end
            
            % 添加相对UWB测量数据
            data.relative_uwb = obj.relative_uwb_data;
            
            save(filename, '-struct', 'data');
        end
    end
end
