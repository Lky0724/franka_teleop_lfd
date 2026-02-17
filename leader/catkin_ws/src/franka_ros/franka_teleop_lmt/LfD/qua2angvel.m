function ang_vel = qua2angvel(raw_quat,dt)

% 假设 raw_quat 是 4xN 矩阵 [w; x; y; z]
nbD = size(raw_quat, 2);
quat = raw_quat;

% === 步骤 A: 连续性处理 (Sign Flipping) ===
% 这一步至关重要！q 和 -q 表示相同的旋转，但如果符号跳变，
% 导数会无穷大。我们需要确保它们在数值上是连续的。
for t = 2:nbD
    if dot(quat(:, t-1), quat(:, t)) < 0
        quat(:, t) = -quat(:, t);
    end
end

% === 步骤 B: 计算四元数微分 (dq) ===
% 使用数值差分近似导数
dq = [diff(quat, 1, 2), zeros(4,1)] ./ dt;

% === 步骤 C: 计算角速度 (omega) ===
ang_vel = zeros(3, nbD);

for t = 1:nbD
    % 提取当前四元数及其导数
    q_curr = quat(:, t)';
    dq_curr = dq(:, t)';
    
    % 计算 q 的共轭
    q_conj = quatconj(q_curr);
    
    % 计算乘积: 2 * dq * q_conj
    % 结果是一个四元数 [0, wx, wy, wz]
    temp_q = 2 * quatmultiply(dq_curr, q_conj);
    
    % 提取虚部作为角速度向量
    ang_vel(:, t) = temp_q(2:4)'; 
end

% 现在可以将 ang_vel 加入到您的数据中进行训练了
% demos(n).Data1 = [pos; quat; lin_vel; ang_vel];