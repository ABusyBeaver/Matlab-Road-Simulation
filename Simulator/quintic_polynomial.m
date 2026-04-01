function v_ref_segment = quintic_polynomial(v0, vf, a0, af, T, dt)
    % quintic_polynomial 生成五次多项式速度参考曲线
    %
    % 输入参数:
    % v0 - 初始速度
    % vf - 终止速度
    % a0 - 初始加速度
    % af - 终止加速度
    % T  - 过渡时间
    % dt - 时间步长
    %
    % 输出参数:
    % v_ref_segment - 速度参考曲线向量

    t = 0:dt:T;
    N = length(t);
    v_ref_segment = zeros(1, N);

    % 五次多项式系数求解
    % 边界条件:
    % t=0: v=v0, a=a0, j=0
    % t=T: v=vf, a=af, j=0

    % 设置矩阵A和向量b以求解五次多项式系数 [c3; c4; c5]
    A = [
        T^3,    T^4,     T^5;
        3*T^2,  4*T^3,   5*T^4;
        6*T,    12*T^2,  20*T^3
    ];

    b = [
        vf - v0 - a0*T;
        af - a0;
        0
    ];

    % 求解系数 [c3; c4; c5]
    coeffs = A \ b;
    c3 = coeffs(1);
    c4 = coeffs(2);
    c5 = coeffs(3);

    % 计算每个时间步的速度
    for i = 1:N
        ti = t(i);
        v_ref_segment(i) = v0 + a0*ti + c3*ti^3 + c4*ti^4 + c5*ti^5;
    end

    % 确保速度非负
    v_ref_segment = abs(v_ref_segment);
end
