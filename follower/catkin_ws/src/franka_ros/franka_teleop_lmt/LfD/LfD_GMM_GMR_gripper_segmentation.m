clear all; close all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RMIL @TU München 2025
% Learning from demonstration using GMM (Segmented + Visualized)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 1. Parameters & Frame Setup
model.nbStates_range = 2:8;                 % Range for BIC check
model.nbD = 100;                            % Number of datapoints (per segment)
model.nbSamples = 3;                        % Number of demonstrations
dataset_path = '../TeleopData/2.13_GMM_5_datasets_with_gripper_pick_and_place/'; 

% % --- Define Global Frames (Start/End) ---
% start_pose = [0.308077	0.000166	0.490933 0 0 3.14];   % x, y, z, roll, pitch, yaw
% end_pose   = [0.476222	0.100887	0.274559 0 0 3.14];   % x, y, z, roll, pitch, yaw
% 
% p(1).A = eul2rotm(start_pose(4:6), 'XYZ');    
% p(1).b = start_pose(1:3)';                 
% p(1).color =  [0.0, 1.0, 0.0];            % Green (Start)
% 
% p(2).A = eul2rotm(end_pose(4:6), 'XYZ');
% p(2).b = end_pose(1:3)';
% p(2).color = [1.0, 0.0, 0.0];             % Red (End)

%% 2. Load and Organize Demonstrations by Segment
fprintf('Loading and segmenting data...\n');
Segments = []; 

% --- MODIFIED: Variables to store global start/end points for frames calculation ---
global_start_points = []; 
global_end_points = [];

for n = 1:model.nbSamples
    fileName = [dataset_path 'follower_' num2str(n) '.txt'];
    
    % Call your segmented loader
    [pos_cells, grip_vals] = loadDemos_segmented(fileName);
    
    num_segments_in_file = length(pos_cells);

    % Capture Global Start (First point of first segment) for Frame Visualization
    if ~isempty(pos_cells)
        first_seg_data = pos_cells{1}'; % Transpose to 3xN
        if ~isempty(first_seg_data)
            global_start_points = [global_start_points, first_seg_data(:,1)];
        end
        
        % Capture Global End (Last point of last segment)
        last_seg_data = pos_cells{end}';
        if ~isempty(last_seg_data)
            global_end_points = [global_end_points, last_seg_data(:,end)];
        end
    end
    
    % Initialize structure on first pass
    if n == 1
        totalSegments = num_segments_in_file;
        for s = 1:totalSegments
            Segments(s).Data = [];      
            Segments(s).Demos = [];     
            Segments(s).StartPoints = []; 
            Segments(s).GripState = grip_vals(s);
        end
    elseif num_segments_in_file ~= totalSegments
        warning('Demo %d has different segment count (%d) than expected (%d).', n, num_segments_in_file, totalSegments);
    end
    
    % Process each segment
    for s = 1:totalSegments
        raw_pos = pos_cells{s}'; % Transpose to [3 x Time]
        
        % Safety check for short segments
        if size(raw_pos, 2) < 2, continue; end
        
        % Spline Interpolation
        pos = spline(1:size(raw_pos,2), raw_pos, linspace(1, size(raw_pos,2), model.nbD));
        
        % Compute Velocity
        vel = [diff(pos,1,2), zeros(3,1)];
        
        % Store Data: Stack [pos; vel]' into the training matrix
        Segments(s).Data = [Segments(s).Data; [pos; vel]']; 
        
        % Store individual demo for plotting
        Segments(s).Demos(n).pos = pos;
        
        % Store start point
        Segments(s).StartPoints(:, n) = pos(:, 1);
    end
end

Learned_Segments = cell(totalSegments, 1);

% Automatically Compute Frame Poses
% Calculate mean position
mean_start_pos = mean(global_start_points, 2)';
mean_end_pos   = mean(global_end_points, 2)';
% Fixed orientation: [0, 0, 3.14] (Roll, Pitch, Yaw)
fixed_orient = [0, 0, 3.14]; 

% Define frames
p(1).A = eul2rotm(fixed_orient, 'XYZ');    
p(1).b = mean_start_pos';                 
p(1).color = [0.0, 1.0, 0.0]; % Green (Start)

p(2).A = eul2rotm(fixed_orient, 'XYZ');
p(2).b = mean_end_pos';
p(2).color = [1.0, 0.0, 0.0]; % Red (End)

fprintf('Frames updated based on data mean.\nStart: [%.2f %.2f %.2f]\nEnd:   [%.2f %.2f %.2f]\n', ...
    mean_start_pos, mean_end_pos);

%% 3. Main Loop: Train, Reproduce, and Visualize
for s = 1:totalSegments
    fprintf('\n--- Processing Segment %d / %d ---\n', s, totalSegments);
    Data = Segments(s).Data; 
    
    %% A. Model Selection (BIC)
    BICs = zeros(1, max(model.nbStates_range));
    BICs(:) = Inf;
    
    for K = model.nbStates_range
        try
            gm = fitgmdist(Data, K, 'Regularizati0.11653onValue', 1e-6, 'Replicates', 3, 'Options', statset('MaxIter', 500));
            BICs(K) = gm.BIC;
        catch
        end
    end
    [~, K_opt] = min(BICs);
    fprintf('Selected K = %d\n', K_opt);
    
    %% B. Fit Final GMM
    options = statset('MaxIter', 1000); 
    GMModel = fitgmdist(Data, K_opt, 'RegularizationValue', 1e-6, ...
                        'Replicates', 5, 'Options', options);
    
    %% C. GMR Reproduction
    x_repro = zeros(3, model.nbD);
    x_repro(:,1) = mean(Segments(s).StartPoints, 2); % Start at mean position
    
    dt = 1; 
    
    for t = 2:model.nbD
        x_t = x_repro(:,t-1);
        v_t = zeros(3,1);
        h = zeros(1, K_opt);
        
        % Responsibilities
        for k = 1:K_opt
            mu_x = GMModel.mu(k,1:3)';
            sigma_x = GMModel.Sigma(1:3,1:3,k);
            h(k) = mvnpdf(x_t', mu_x', sigma_x);
        end
        h = h / (sum(h) + realmin);
        
        % Expected Velocity
        for k = 1:K_opt
            mu_x = GMModel.mu(k,1:3)';
            mu_v = GMModel.mu(k,4:6)';
            sigma_x = GMModel.Sigma(1:3,1:3,k);
            sigma_vx = GMModel.Sigma(4:6,1:3,k);
            
            vel = mu_v + sigma_vx / sigma_x * (x_t - mu_x);
            v_t = v_t + h(k) * vel;
        end
        x_repro(:,t) = x_repro(:,t-1) + v_t * dt;
    end

    Learned_Segments{s} = x_repro;
    
    %% D. Save Learned CSV
    x_interp = spline(1:size(x_repro,2), x_repro, linspace(1,size(x_repro,2), model.nbD*120));
    out_filename = sprintf('%slearned_seg_%d.csv', dataset_path, s);

    % 1. Write the single gripper value (0 or 1)
    dlmwrite(out_filename, Segments(s).GripState, 'delimiter', ',');
    
    % 2. Append the trajectory data (Matrix) below it
    dlmwrite(out_filename, x_interp, '-append', 'delimiter', ',');

    %% E. VISUALIZATION (Updated)
    figure('Name', sprintf('Segment %d Analysis', s), 'Color', 'w');
    hold on; grid on;
    
    % 1. Plot Input Demos (Gray Lines)
    h_demos = [];
    for n = 1:model.nbSamples
        if n <= length(Segments(s).Demos) && ~isempty(Segments(s).Demos(n).pos)
            h_d = plot3(Segments(s).Demos(n).pos(1,:), Segments(s).Demos(n).pos(2,:), Segments(s).Demos(n).pos(3,:), ...
                  'Color', [0.7 0.7 0.7], 'LineWidth', 1.5);
            h_demos = [h_demos, h_d]; % Collect handles
        end
    end
    
    % 2. Plot Frames (Green/Red Axes)
    h_frames = plotFrames3D(p);
    
    % 3. Plot GMM Ellipsoids
    % Extract Position Mu (1:3) and Position Sigma (1:3, 1:3)
    % Transpose Mu to match GMM3D_plot expectation [Dimensions x States]
    % Only pass 3 arguments: Mean, Sigma, and Color

    % GMM3D_plot(GMModel.mu(:,1:3)', GMModel.Sigma(1:3,1:3,:), 1); % plot
    % with GMM Ellipsoids (slow)
    GMM3D_plot(GMModel.mu(:,1:3)', GMModel.Sigma(1:3,1:3,:), [0 0 1]); % plot without GMM Ellipsoids (fast)
    
    % 4. Plot Reproduced Trajectory (Black Dashed)
    h_repro = plot3(x_repro(1,:), x_repro(2,:), x_repro(3,:), 'k--', 'LineWidth', 3.0);
    
    % Formatting
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    title(sprintf('Segment %d (Gripper: %d) | K=%d', s, Segments(s).GripState(1), K_opt));
    view(3);
    
    % Legend logic (safe check)
    if ~isempty(h_demos)
        legend([h_demos(1), h_repro], 'Demonstrations', 'Reproduction', 'Location', 'best');
    end
    
    drawnow;
end

disp('Done.');

%% 4. Global Concatenation & Visualization (Updated with Gripper State)
figure('Name', 'Global Concatenated Trajectories', 'Color', 'w');

% --- Part A: Stitch and Plot Demonstrations ---
subplot(1,2,1); 
hold on; grid on; 
% --- MODIFIED: Added subtitle explaining line styles ---
title({'Concatenated Demonstrations', 'Solid: Closed (1) | Dashed: Open (0)'}); 
plotFrames3D(p); 
colors = lines(model.nbSamples);
h_demos = []; 

for n = 1:model.nbSamples
    first_seg_handle = [];
    
    % --- MODIFIED: Loop through segments to plot each with correct line style ---
    for s = 1:totalSegments
        if n <= length(Segments(s).Demos) && ~isempty(Segments(s).Demos(n).pos)
            
            % Determine Line Style based on Gripper
            if Segments(s).GripState == 1
                ls = '-'; % Closed
            else
                ls = '--';  % Open
            end
            
            % Plot Segment
            h_seg = plot3(Segments(s).Demos(n).pos(1,:), Segments(s).Demos(n).pos(2,:), Segments(s).Demos(n).pos(3,:), ...
                  'Color', colors(n,:), 'LineWidth', 1.5, 'LineStyle', ls);
            
            % Capture handle of the first segment for the legend
            if isempty(first_seg_handle)
                first_seg_handle = h_seg;
            end
            
            % Junction Dot
            plot3(Segments(s).Demos(n).pos(1,end), Segments(s).Demos(n).pos(2,end), Segments(s).Demos(n).pos(3,end), ...
                  'o', 'Color', colors(n,:), 'MarkerSize', 3, 'HandleVisibility','off');
        end
    end
    % --------------------------------------------------------------------------
    
    if ~isempty(first_seg_handle)
        h_demos = [h_demos, first_seg_handle]; 
    end
end
xlabel('x'); ylabel('y'); zlabel('z'); view(3);
legend(h_demos, 'Demo 1', 'Demo 2', 'Demo 3', 'Location', 'best'); 

% --- Part B: Stitch and Plot Learned Trajectory ---
subplot(1,2,2); 
hold on; grid on; 
% --- MODIFIED: Added subtitle explaining line styles ---
title({'Concatenated Reproduction', 'Solid: Closed (1) | Dashed: Open (0)'}); 
plotFrames3D(p); 

seg_colors = lines(totalSegments); 
h_learned_segs = []; 
legend_labels = {}; 

for s = 1:totalSegments
    % --- MODIFIED: Determine Line Style based on Gripper ---
    if Segments(s).GripState == 1
        ls = '-'; % Closed
    else
        ls = '--';  % Open
    end
    
    % Plot segment with Gripper-dependent LineStyle
    h_seg = plot3(Learned_Segments{s}(1,:), Learned_Segments{s}(2,:), Learned_Segments{s}(3,:), ...
          'Color', seg_colors(s,:), 'LineWidth', 3.0, 'LineStyle', ls);
    % ------------------------------------------------------
    
    h_learned_segs = [h_learned_segs, h_seg];
    legend_labels{end+1} = sprintf('Seg %d', s);
    
    plot3(Learned_Segments{s}(1,end), Learned_Segments{s}(2,end), Learned_Segments{s}(3,end), ...
          'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 5, 'HandleVisibility', 'off'); 
    
    start_pt = Learned_Segments{s}(:,1);
    text(start_pt(1), start_pt(2), start_pt(3)+0.02, sprintf('S%d', s), ...
        'FontSize', 8, 'BackgroundColor', 'w', 'EdgeColor', 'k');
end

xlabel('x'); ylabel('y'); zlabel('z'); view(3);
legend(h_learned_segs, legend_labels, 'Location', 'best');

disp('All processing and global visualization complete.');