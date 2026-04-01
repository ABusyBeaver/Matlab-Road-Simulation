function [left_border, right_border] = generate_boundaries(centerline, road_half_width)
    % 生成车道的左右边界
    % centerline: [x, y] 中心线轨迹点
    % road_half_width: 车道半宽度

    num_points = size(centerline, 1);
    left_border = zeros(num_points, 2);
    right_border = zeros(num_points, 2);

    for i = 1:num_points
        if i == 1
            % 计算第一个点的方向
            direction = centerline(i+1, :) - centerline(i, :);
        elseif i == num_points
            % 计算最后一个点的方向
            direction = centerline(i, :) - centerline(i-1, :);
        else
            % 计算中间点的方向，取前后方向的平均
            dir_prev = centerline(i, :) - centerline(i-1, :);
            dir_next = centerline(i+1, :) - centerline(i, :);
            direction = (dir_prev + dir_next) / 2;
        end

        % 归一化方向向量
        direction = direction / norm(direction);

        % 计算法线向量
        normal = [-direction(2), direction(1)];

        % 计算左右边界点
        left_border(i, :) = centerline(i, :) + road_half_width * normal;
        right_border(i, :) = centerline(i, :) - road_half_width * normal;
    end
end
