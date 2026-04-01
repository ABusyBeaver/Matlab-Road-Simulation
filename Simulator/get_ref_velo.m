function v_ref = get_ref_velo(v_d, dt, offset)
    % get_ref_velo 生成整个仿真的速度参考向量，并应用时间偏移
    %
    % 输入参数:
    % v_d    - 基础期望速度 (m/s)
    % dt     - 时间步长 (s)
    % offset - 时间偏移 (s)，用于多车仿真中的起始延迟
    %
    % 输出参数:
    % v_ref  - 速度参考向量

    % 定义道路分段
    segments_1 = {
        'straight', [offset, -0.525; 2.1, -0.525];
        'arc',      [2.1, -0.525; 2.1, 0.525; 2.1, 0];
        'straight', [2.1, 0.525; 0, 0.525];
        'straight', [0, 0.525; -2.1, 0.525];
        'arc',      [-2.1, 0.525; -2.1, -0.525; -2.1, 0];
        'straight', [-2.1, -0.525; 0, -0.525];
    };

    % 定义每个分段的期望速度
    % 直线：v_d，弧线：v_d / 2
    desired_speeds = cell(length(segments_1), 1);
    for i = 1:length(segments_1)
        if strcmp(segments_1{i,1}, 'straight')
            desired_speeds{i} = v_d;
        elseif strcmp(segments_1{i,1}, 'arc')
            desired_speeds{i} = v_d / 2;
        end
    end

    % 计算每个分段的长度
    segment_lengths = zeros(length(segments_1), 1);
    for i = 1:length(segments_1)
        points = segments_1{i,2};
        if strcmp(segments_1{i,1}, 'straight')
            % 直线段长度
            segment_lengths(i) = norm(points(2,:) - points(1,:));
        elseif strcmp(segments_1{i,1}, 'arc')
            % 弧线段长度，假设圆心在分段的第三个点
            center = points(3, :);
            radius = norm(points(1,:) - center);
            % 计算两个端点与圆心之间的夹角
            vec1 = points(1,:) - center;
            vec2 = points(2,:) - center;
            dot_product = dot(vec1, vec2) / (norm(vec1) * norm(vec2));
            % 处理数值误差导致的超出范围
            dot_product = max(min(dot_product, 1), -1);
            theta = acos(dot_product);
            segment_lengths(i) = radius * theta;
        end
    end

    % 定义过渡时间
    transition_time = 1; % 1秒的过渡时间
    transition_steps = ceil(transition_time / dt);

    % 计算总仿真时间
    total_time = sum(segment_lengths ./ cell2mat(desired_speeds)) + (length(segments_1)-1)*transition_time;
    N = ceil(total_time / dt);
    
    % 考虑时间偏移，增加偏移对应的步数
    offset_steps = ceil(offset / dt);
    N_total = N + offset_steps; % 总步数包括偏移

    v_ref = zeros(1, N_total); % 初始化速度参考向量，预填充偏移部分为0

    current_index = offset_steps + 1; % 开始填充速度参考的索引位置
    v0 = 0; % 初始速度
    a0 = 0; % 初始加速度

    for i = 1:length(segments_1)
        vi = desired_speeds{i};
        Li = segment_lengths(i);
        Ti = Li / vi; % 分段所需时间

        % 生成分段内的速度参考
        v_segment = quintic_polynomial(v0, vi, a0, 0, Ti, dt);
        steps_segment = length(v_segment);
        end_index = current_index + steps_segment - 1;
        if end_index > N_total
            end_index = N_total;
            v_segment = v_segment(1:(end_index - current_index +1));
        end
        v_ref(current_index:end_index) = v_segment;
        current_index = end_index + 1;

        % 生成过渡速度参考
        if i < length(segments_1)
            vi_next = desired_speeds{i+1};
            delta_v = vi_next - vi;
            if delta_v ~= 0
                v_transition = quintic_polynomial(vi, vi_next, 0, 0, transition_time, dt)';
            else
                v_transition = vi * ones(transition_steps, 1);
            end
        end

        % Assign transition velocity to v_ref
        if i < length(segments_1)
            steps_transition = length(v_transition);
            end_index = current_index + steps_transition - 1;
            if end_index > N_total
                end_index = N_total;
                v_transition = v_transition(1:(end_index - current_index +1));
            end
            v_ref(current_index:end_index) = v_transition;
            current_index = end_index +1;
            v0 = vi_next; % 更新下一个分段的初始速度
        end
    end

    % 处理任何剩余的速度参考
    if current_index <= N_total
        v_ref(current_index:N_total) = v_ref(current_index-1);
    end
end
