clear all; close all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RMIL @TU München 2026
% Learning from demonstration using Time-Driven GMR
% Features: Segmented BIC, Time-driven regression, 
%           Connectivity markers, and full legends.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 1. Parameters & Frame Setup
model.nbStates_range = 2:8;                 % BIC 检查的状态数范围
model.nbD = 100;                            % 每个片段的采样点数
model.nbSamples = 4;                        % 演示次数
dataset_path = '../TeleopData/2.13_GMM_5_datasets_with_gripper_pick_and_place/'; 

%% 2. Load and Organize Demonstrations by Segment
fprintf('Loading and segmenting data...\n');
Segments = []; 

global_start_points = []; 
global_end_points = [];

for n = 1:model.nbSamples
    fileName = [dataset_path 'follower_' num2str(n) '.txt'];
    [pos_cells, grip_vals] = loadDemos_segmented(fileName); % 加载分段数据
    num_segments_in_file = length(pos_cells);

    if ~isempty(pos_cells)
        first_seg_data = pos_cells{1}'; 
        if ~isempty(first_seg_data), global_start_points = [global_start_points, first_seg_data(:,1)]; end
        last_seg_data = pos_cells{end}';
        if ~isempty(last_seg_data), global_end_points = [global_end_points, last_seg_data(:,end)]; end
    end
    
    if n == 1
        totalSegments = num_segments_in_file;
        for s = 1:totalSegments
            Segments(s).Data = [];      
            Segments(s).Demos = [];     
            Segments(s).GripState = grip_vals(s);
        end
    end
    
    for s = 1:totalSegments
        raw_pos = pos_cells{s}'; 
        if size(raw_pos, 2) < 2, continue; end
        
        % 样条插值保证点数一致
        pos = spline(1:size(raw_pos,2), raw_pos, linspace(1, size(raw_pos,2), model.nbD));
        
        % 构造时间轴作为输入变量
        t_vec = linspace(1, model.nbD, model.nbD);
        
        % 存储数据为 [时间; 位置] 用于时间驱动模型
        Segments(s).Data = [Segments(s).Data; [t_vec; pos]']; 
        Segments(s).Demos(n).pos = pos;
    end
end

% 计算全局可视化参考帧
mean_start_pos = mean(global_start_points, 2)';
mean_end_pos   = mean(global_end_points, 2)';
fixed_orient = [0, 0, 3.14]; 
p(1).A = eul2rotm(fixed_orient, 'XYZ'); p(1).b = mean_start_pos'; p(1).color = [0.0, 1.0, 0.0];
p(2).A = eul2rotm(fixed_orient, 'XYZ'); p(2).b = mean_end_pos'; p(2).color = [1.0, 0.0, 0.0];

Learned_Segments = cell(totalSegments, 1);

%% 3. Main Loop: Train, Reproduce, and Visualize
for s = 1:totalSegments
    fprintf('\n--- Processing Segment %d / %d ---\n', s, totalSegments);
    Data = Segments(s).Data; % 数据格式为 [t, x, y, z]
    
    %% A. Model Selection (BIC)
    BICs = zeros(1, max(model.nbStates_range));
    BICs(:) = Inf;
    for K = model.nbStates_range
        try
            gm = fitgmdist(Data, K, 'RegularizationValue', 1e-6, 'Replicates', 3, 'Options', statset('MaxIter', 500));
            BICs(K) = gm.BIC;
        catch
        end
    end
    [~, K_opt] = min(BICs);
    fprintf('Selected K = %d\n', K_opt);
    
    %% B. Fit Final GMM
    GMModel = fitgmdist(Data, K_opt, 'RegularizationValue', 1e-6, 'Replicates', 5, 'Options', statset('MaxIter', 1000));
    
    %% C. GMR Reproduction (Time-Driven P(Pos | Time))
    x_repro = zeros(3, model.nbD);
    t_query = linspace(1, model.nbD, model.nbD);
    
    for t = 1:model.nbD
        curr_t = t_query(t);
        h = zeros(1, K_opt);
        
        % 1. 计算基于时间的响应度
        for k = 1:K_opt
            mu_t = GMModel.mu(k,1);
            sigma_t = GMModel.Sigma(1,1,k);
            h(k) = GMModel.ComponentProportion(k) * mvnpdf(curr_t, mu_t, sigma_t);
        end
        h = h / (sum(h) + realmin);
        
        % 2. 计算条件均值
        for k = 1:K_opt
            mu_t = GMModel.mu(k,1);
            mu_pos = GMModel.mu(k,2:4)';
            sigma_t = GMModel.Sigma(1,1,k);
            sigma_pos_t = GMModel.Sigma(2:4,1,k);
            
            % GMR 条件分布公式
            pos_k = mu_pos + sigma_pos_t / sigma_t * (curr_t - mu_t);
            x_repro(:,t) = x_repro(:,t) + h(k) * pos_k;
        end
    end

    Learned_Segments{s} = x_repro;
    
    %% D. Save Learned CSV
    x_interp = spline(1:size(x_repro,2), x_repro, linspace(1,size(x_repro,2), model.nbD*100));
    out_filename = sprintf('%slearned_seg_%d.csv', dataset_path, s);
    dlmwrite(out_filename, Segments(s).GripState, 'delimiter', ',');
    dlmwrite(out_filename, x_interp, '-append', 'delimiter', ',');

    %% E. 单片段可视化分析
    figure('Name', sprintf('Segment %d Analysis', s), 'Color', 'w'); hold on; grid on;
    h_demos = [];
    for n = 1:model.nbSamples
        if n <= length(Segments(s).Demos) && ~isempty(Segments(s).Demos(n).pos)
            h_d = plot3(Segments(s).Demos(n).pos(1,:), Segments(s).Demos(n).pos(2,:), Segments(s).Demos(n).pos(3,:), ...
                  'Color', [0.7 0.7 0.7], 'LineWidth', 1.5);
            h_demos = [h_demos, h_d];
        end
    end
    plotFrames3D(p);
    GMM3D_plot(GMModel.mu(:,2:4)', GMModel.Sigma(2:4,2:4,:), [0 0 1]); 
    h_repro = plot3(x_repro(1,:), x_repro(2,:), x_repro(3,:), 'k--', 'LineWidth', 3.0);
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    title(sprintf('Segment %d (Gripper: %d) | Time-Driven K=%d', s, Segments(s).GripState(1), K_opt));
    view(3);
    if ~isempty(h_demos), legend([h_demos(1), h_repro], 'Demonstrations', 'Reproduction', 'Location', 'best'); end
    drawnow;
end

%% 4. Global Concatenation & Visualization
figure('Name', 'Global Concatenated Trajectories', 'Color', 'w', 'Position', [100 100 1200 500]);

% --- 左图：拼接所有演示轨迹 ---
subplot(1,2,1); hold on; grid on; title('Concatenated Demonstrations'); 
plotFrames3D(p); colors = lines(model.nbSamples);
h_demo_list = [];
for n = 1:model.nbSamples
    first_seg_h = [];
    for s = 1:totalSegments
        if n <= length(Segments(s).Demos) && ~isempty(Segments(s).Demos(n).pos)
            ls = '--'; if Segments(s).GripState == 1, ls = '-'; end
            h_line = plot3(Segments(s).Demos(n).pos(1,:), Segments(s).Demos(n).pos(2,:), Segments(s).Demos(n).pos(3,:), ...
                  'Color', colors(n,:), 'LineWidth', 1.5, 'LineStyle', ls);
            if s == 1, first_seg_h = h_line; end
        end
    end
    h_demo_list = [h_demo_list, first_seg_h];
end
view(3); legend(h_demo_list, arrayfun(@(i) sprintf('Demo %d', i), 1:model.nbSamples, 'UniformOutput', false), 'Location', 'best');

% --- 右图：拼接时间驱动复现轨迹 ---
subplot(1,2,2); hold on; grid on; title('Concatenated Time-Driven Reproduction'); 
plotFrames3D(p); seg_colors = lines(totalSegments);
h_repro_list = [];
for s = 1:totalSegments
    ls = '--'; if Segments(s).GripState == 1, ls = '-'; end
    h_seg = plot3(Learned_Segments{s}(1,:), Learned_Segments{s}(2,:), Learned_Segments{s}(3,:), ...
          'Color', seg_colors(s,:), 'LineWidth', 3.0, 'LineStyle', ls);
    h_repro_list = [h_repro_list, h_seg];
    
    % 连接性标记：起点(o)与终点(x)
    start_pt = Learned_Segments{s}(:,1);
    end_pt   = Learned_Segments{s}(:,end);
    plot3(start_pt(1), start_pt(2), start_pt(3), 'o', 'Color', seg_colors(s,:), 'MarkerSize', 8, 'LineWidth', 2, 'HandleVisibility', 'off');
    plot3(end_pt(1), end_pt(2), end_pt(3), 'x', 'Color', seg_colors(s,:), 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off');
    
    text(start_pt(1), start_pt(2), start_pt(3)+0.02, sprintf('S%d', s), 'FontSize', 8, 'BackgroundColor', 'w');
end
view(3); legend(h_repro_list, arrayfun(@(i) sprintf('Segment %d', i), 1:totalSegments, 'UniformOutput', false), 'Location', 'best');

disp('All processing complete.');