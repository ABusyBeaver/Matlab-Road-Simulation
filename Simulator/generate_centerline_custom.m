function centerline = generate_centerline_custom(y_offset)
    % 通用的车道中心线轨迹生成函数
    % 输入:
    %   y_offset - [y_start, y_end] y 方向的偏移量
    % 输出:
    %   centerline - [x, y] 坐标点的矩阵

    % 参数
    num_straight = 20; % 直线段点数
    num_arc = 20;      % 圆弧段点数

    % 确定起点 y 坐标
    p_start = [0, y_offset(1)];

    % 1. 直线段：从起点到 (2.1, y_offset(1))
    straight1_x = linspace(p_start(1), 2.1, num_straight)';
    straight1_y = linspace(p_start(2), y_offset(1), num_straight)';
    straight1 = [straight1_x, straight1_y];

    % 2. 圆弧段：从 (2.1, y_offset(1)) 到 (2.1, y_offset(2))，圆心在 (2.1, 0)
    theta = linspace(-pi/2, pi/2, num_arc)';
    radius = abs(y_offset(2) - 0); % 假设圆心在 y=0
    arc1_x = 2.1 + radius * cos(theta);
    arc1_y = 0 + radius * sin(theta);
    arc1 = [arc1_x, arc1_y];

    % 3. 直线段：从 (2.1, y_offset(2)) 到 (-2.1, y_offset(2))
    straight2_x = linspace(2.1, -2.1, num_straight)';
    straight2_y = repmat(y_offset(2), num_straight, 1);
    straight2 = [straight2_x, straight2_y];

    % 4. 圆弧段：从 (-2.1, y_offset(2)) 到 (-2.1, y_offset(1))，圆心在 (-2.1, 0)
    theta2 = linspace(pi/2, 3*pi/2, num_arc)';
    arc2_x = -2.1 + radius * cos(theta2);
    arc2_y = 0 + radius * sin(theta2);
    arc2 = [arc2_x, arc2_y];

    % 5. 直线段：从 (-2.1, y_offset(1)) 回到起点 (0, y_offset(1))
    straight3_x = linspace(-2.1, 0, num_straight)';
    straight3_y = repmat(y_offset(1), num_straight, 1);
    straight3 = [straight3_x, straight3_y];

    % 合并所有段
    centerline = [straight1; arc1; straight2; arc2; straight3];
end
