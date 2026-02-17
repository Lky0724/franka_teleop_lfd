clear all; close all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RMIL @TU München 2026
% Time-Driven Segmented TP-GMM / GMR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 1. Parameters & Frame Setup
model.nbStates = 5;                 % BIC 检查的状态数范围
model.nbD = 100;                            % 每个片段的采样点数
model.nbSamples = 4;                        % 演示次数
model.dt = 0.1;
model.reg = 1e-7;                           % 较小的正则化以保证端点精度
model.nbVar = 4;                            % 状态：[t, x, y, z] (时间驱动)
model.nbFrames = 2;                         % 起点帧和终点帧
dataset_path = '../TeleopData/2.13_GMM_5_datasets_with_gripper_pick_and_place/'; 

%% 2. Load and Organize Demonstrations by Segment
fprintf('Loading and segmenting data...\n');
Segments = []; 

for n = 1:model.nbSamples
    fileName = [dataset_path 'follower_' num2str(n) '.txt'];
    % 使用您提供的分段加载函数
    [pos_cells, grip_vals] = loadDemos_segmented(fileName); 
    totalSegments = length(pos_cells);

    if n == 1
        for s = 1:totalSegments
            Segments(s).GripState = grip_vals(s);
            Segments(s).StartPoints = [];
            Segments(s).EndPoints = [];
        end
    end
    
    for s = 1:totalSegments
        raw_pos = pos_cells{s}'; 
        % 样条插值保证点数一致
        pos = spline(1:size(raw_pos,2), raw_pos, linspace(1, size(raw_pos,2), model.nbD));
        
        Segments(s).Demos(n).pos = pos;
        Segments(s).StartPoints(:, n) = pos(:, 1);
        Segments(s).EndPoints(:, n) = pos(:, end);
    end
end

Learned_Segments = cell(totalSegments, 1);
fixed_orient = [0, 0, 3.14]; % 固定坐标系朝向

%% 3. Main Loop: Train, Reproduce, and Visualize per Segment
for s = 1:totalSegments
    fprintf('\n--- Processing Segment %d / %d (Time-Driven TP-GMM) ---\n', s, totalSegments);
    
    % --- Step A: 为当前片段定义局部参考帧 ---
    m_start = mean(Segments(s).StartPoints, 2);
    m_end   = mean(Segments(s).EndPoints, 2);
    
    p_task(1).A = eul2rotm(fixed_orient, 'XYZ'); p_task(1).b = m_start; p_task(1).color = [0 1 0]; % 绿:起点
    p_task(2).A = eul2rotm(fixed_orient, 'XYZ'); p_task(2).b = m_end;   p_task(2).color = [1 0 0]; % 红:终点
    Segments(s).p_task = p_task;

    % --- Step B: 将演示数据转换到局部坐标系 [t; x; y; z] ---
    t_axis = linspace(1, model.nbD, model.nbD);
    TPGMM_Data = zeros(model.nbVar, model.nbFrames, model.nbSamples * model.nbD);
    
    for n = 1:model.nbSamples
        for m = 1:model.nbFrames
            for t = 1:model.nbD
                pos_g = Segments(s).Demos(n).pos(:,t);
                % 空间坐标进行旋转平移，时间坐标保持不变
                x_local = p_task(m).A' * (pos_g - p_task(m).b);
                TPGMM_Data(:,m,(n-1)*model.nbD + t) = [t_axis(t); x_local];
            end
        end
    end
    
    % --- Step C: 训练最终 TP-GMM ---
    seg_model = model;
    seg_model = trainTPGMM_tensor(TPGMM_Data, seg_model);
    
    % --- Step D: TP-GMR 复现 (根据时间回归位置 P(x|t)) ---
    x_repro = zeros(3, model.nbD);
    
    for t = 1:model.nbD
        curr_t = t_axis(t);
        h = zeros(seg_model.nbStates, 1);
        mu_x_all = zeros(3, seg_model.nbStates);

        for i = 1:seg_model.nbStates
            Sigma_sum = zeros(4); Mu_sum = zeros(4,1);
            for m = 1:model.nbFrames
                % 构造增强矩阵 A_aug (1维时间 + 3维空间)
                A_aug = blkdiag(1, p_task(m).A);
                b_aug = [0; p_task(m).b];

                Mu_g = A_aug * seg_model.Mu(:,m,i) + b_aug;
                Sigma_g = A_aug * seg_model.Sigma(:,:,m,i) * A_aug';
                
                invSig = inv(Sigma_g + model.reg*eye(4));
                Sigma_sum = Sigma_sum + invSig;
                Mu_sum = Mu_sum + invSig * Mu_g;
            end
            Sigma_joint = inv(Sigma_sum + model.reg*eye(4));
            Mu_joint = Sigma_joint * Mu_sum;
            
            % GMR 条件估计: P(x | t)
            % 索引1是时间(In)，索引2:4是位置(Out)
            mu_xt = Mu_joint(2:4) + Sigma_joint(2:4,1)/Sigma_joint(1,1)*(curr_t - Mu_joint(1));
            h(i) = seg_model.Priors(i) * mvnpdf(curr_t, Mu_joint(1), Sigma_joint(1,1));
            mu_x_all(:,i) = mu_xt;
        end
        h = h / (sum(h) + realmin);
        x_repro(:,t) = sum(mu_x_all .* h', 2); % 直接得到位置
    end
    Learned_Segments{s} = x_repro;
    
    % --- Step E: 存储 CSV ---
    x_interp = spline(1:size(x_repro,2), x_repro, linspace(1,size(x_repro,2), model.nbD*120));
    out_filename = sprintf('%slearned_seg_%d.csv', dataset_path, s);
    dlmwrite(out_filename, Segments(s).GripState, 'delimiter', ',');
    dlmwrite(out_filename, x_interp, '-append', 'delimiter', ',');

    % --- Step F: 单个片段可视化 ---
    figure('Name', sprintf('Segment %d Analysis', s), 'Color', 'w'); hold on; grid on;
    h_d_list = [];
    for n = 1:model.nbSamples
        h_d = plot3(Segments(s).Demos(n).pos(1,:), Segments(s).Demos(n).pos(2,:), Segments(s).Demos(n).pos(3,:), 'Color', [0.7 0.7 0.7]);
        if n==1, h_d_list = h_d; end
    end
    h_f = plotFrames3D(p_task);
    h_r = plot3(x_repro(1,:), x_repro(2,:), x_repro(3,:), 'k--', 'LineWidth', 2.5);
    legend([h_d_list, h_r, h_f(1), h_f(4)], 'Demos', 'TP-GMR Repro', 'Start Frame', 'End Frame');
    title(sprintf('Segment %d (Gripper: %d)', s, Segments(s).GripState)); view(3);
end

%% 4. 全局对比可视化
figure('Name', 'Global Comparative Analysis', 'Color', 'w', 'Position', [100 100 1300 500]);

% 左图：原始演示
subplot(1,2,1); hold on; grid on;
title({'Original Demonstrations', 'Solid: Closed (1) | Dashed: Open (0)'});
colors = lines(model.nbSamples);
h_demo_leg = [];
for n = 1:model.nbSamples
    for s = 1:totalSegments
        ls = '--'; if Segments(s).GripState == 1, ls = '-'; end
        h_l = plot3(Segments(s).Demos(n).pos(1,:), Segments(s).Demos(n).pos(2,:), Segments(s).Demos(n).pos(3,:), ...
              'Color', colors(n,:), 'LineStyle', ls, 'LineWidth', 1.2);
        if s == 1, h_demo_leg(n) = h_l; end
    end
end
view(3); xlabel('x'); ylabel('y'); zlabel('z');
legend(h_demo_leg, arrayfun(@(x) sprintf('Demo %d', x), 1:model.nbSamples, 'UniformOutput', false));

% 右图：TP-GMM 复现
subplot(1,2,2); hold on; grid on;
title({'Time-Driven TP-GMM Reproduction', 'Points: Start (G) / End (R)'});
seg_colors = lines(totalSegments);
h_seg_leg = [];
for s = 1:totalSegments
    ls = '--'; if Segments(s).GripState == 1, ls = '-'; end
    h_s = plot3(Learned_Segments{s}(1,:), Learned_Segments{s}(2,:), Learned_Segments{s}(3,:), ...
          'LineWidth', 3, 'LineStyle', ls, 'Color', seg_colors(s,:));
    h_seg_leg(s) = h_s;
    
    % 将 Frame 简化为点以保持界面整洁
    plot3(Segments(s).p_task(1).b(1), Segments(s).p_task(1).b(2), Segments(s).p_task(1).b(3), 'go','MarkerFaceColor','g','MarkerSize',4,'HandleVisibility','off');
    plot3(Segments(s).p_task(2).b(1), Segments(s).p_task(2).b(2), Segments(s).p_task(2).b(3), 'ro','MarkerFaceColor','r','MarkerSize',4,'HandleVisibility','off');
end
view(3); xlabel('x'); ylabel('y'); zlabel('z');
legend(h_seg_leg, arrayfun(@(x) sprintf('Seg %d', x), 1:totalSegments, 'UniformOutput', false));

disp('Done.');