clear all; close all; clc;

%% 1. Parameters
model.nbStates = 6;         % Number of states for TP-GMM
model.nbD = 100;            % Datapoints per segment
model.nbSamples = 4;        % Number of demonstrations
model.dt = 0.1;             % Time step
model.reg = 1e-4;           % Regularization factor
model.nbVar = 6;            % State: [x, y, z, vx, vy, vz]
model.nbFrames = 2;         % Start and End frames
dataset_path = '../TeleopData/2.12_GMM_5_datasets_with_gripper_pick_and_place/'; 

%% 2. Load and Segment Data
fprintf('Loading and segmenting data...\n');
Segments = []; 

for n = 1:model.nbSamples
    fileName = [dataset_path 'follower_' num2str(n) '.txt'];
    [pos_cells, grip_vals] = loadDemos_segmented(fileName); % Segmented loader
    totalSegments = length(pos_cells);
    
    if n == 1
        for s = 1:totalSegments
            Segments(s).Data1 = []; 
            Segments(s).GripState = grip_vals(s);
            Segments(s).StartPoints = [];
            Segments(s).EndPoints = [];
        end
    end
    
    for s = 1:totalSegments
        raw_pos = pos_cells{s}';
        % Spline interpolation for consistent length
        pos = spline(1:size(raw_pos,2), raw_pos, linspace(1, size(raw_pos,2), model.nbD));
        vel = [diff(pos,1,2), zeros(3,1)] ./ model.dt; % Compute velocity
        
        Segments(s).Demos(n).pos = pos;
        Segments(s).Demos(n).vel = vel;
        Segments(s).Data1(:,:,n) = [pos; vel]; 
        Segments(s).StartPoints(:, n) = pos(:, 1);
        Segments(s).EndPoints(:, n) = pos(:, end);
    end
end

Learned_Segments = cell(totalSegments, 1);
fixed_orient = [0, 0, 3.14]; % Standard fixed orientation

%% 3. Main Loop: Train TP-GMM per Segment
for s = 1:totalSegments
    fprintf('\n--- TP-GMM Segment %d / %d ---\n', s, totalSegments);
    
    % Define Local Frames for this segment
    m_start = mean(Segments(s).StartPoints, 2);
    m_end   = mean(Segments(s).EndPoints, 2);
    
    p_task(1).A = eul2rotm(fixed_orient, 'XYZ'); p_task(1).b = m_start; p_task(1).color = [0 1 0]; % Green Start
    p_task(2).A = eul2rotm(fixed_orient, 'XYZ'); p_task(2).b = m_end;   p_task(2).color = [1 0 0]; % Red End
    Segments(s).p_task = p_task; 
    
    % Transform Data to Local Frame Coordinates
    TPGMM_Data = zeros(model.nbVar, model.nbFrames, model.nbSamples * model.nbD);
    for n = 1:model.nbSamples
        for m = 1:model.nbFrames
            for t = 1:model.nbD
                data_g = Segments(s).Data1(:,t,n);
                TPGMM_Data(1:3,m,(n-1)*model.nbD + t) = p_task(m).A' * (data_g(1:3) - p_task(m).b);
                TPGMM_Data(4:6,m,(n-1)*model.nbD + t) = p_task(m).A' * data_g(4:6);
            end
        end
    end
    
    % Train TP-GMM
    seg_model = trainTPGMM_tensor(TPGMM_Data, model);
    
    % TP-GMR Rollout (v|x)
    x_repro = zeros(3, model.nbD);
    x_repro(:,1) = m_start; 
    
    for t = 2:model.nbD
        x_curr = x_repro(:,t-1);
        h = zeros(seg_model.nbStates, 1);
        mu_v_all = zeros(3, seg_model.nbStates);

        for i = 1:seg_model.nbStates
            Sigma_sum = zeros(6); Mu_sum = zeros(6,1);
            for m = 1:model.nbFrames
                A_aug = blkdiag(p_task(m).A, p_task(m).A);
                b_aug = [p_task(m).b; zeros(3,1)];
                Mu_g = A_aug * seg_model.Mu(:,m,i) + b_aug;
                Sigma_g = A_aug * seg_model.Sigma(:,:,m,i) * A_aug';
                
                invSig = inv(Sigma_g + model.reg*eye(6));
                Sigma_sum = Sigma_sum + invSig;
                Mu_sum = Mu_sum + invSig * Mu_g;
            end
            Sigma_joint = inv(Sigma_sum + model.reg*eye(6));
            Mu_joint = Sigma_joint * Mu_sum;
            
            % GMR Condition v|x
            mu_vx = Mu_joint(4:6) + Sigma_joint(4:6,1:3)/Sigma_joint(1:3,1:3)*(x_curr - Mu_joint(1:3));
            h(i) = seg_model.Priors(i) * mvnpdf(x_curr, Mu_joint(1:3), Sigma_joint(1:3,1:3));
            mu_v_all(:,i) = mu_vx;
        end
        h = h / (sum(h) + realmin);
        x_repro(:,t) = x_repro(:,t-1) + (sum(mu_v_all .* h', 2)) * model.dt;
    end
    Learned_Segments{s} = x_repro;

    % Store CSV (Gripper state followed by XYZ trajectory)
    x_interp = spline(1:size(x_repro,2), x_repro, linspace(1,size(x_repro,2), model.nbD*120));
    out_filename = sprintf('%slearned_seg_%d.csv', dataset_path, s);
    dlmwrite(out_filename, Segments(s).GripState, 'delimiter', ',');
    dlmwrite(out_filename, x_interp, '-append', 'delimiter', ',');

    % Individual Segment Visualization
    figure('Name', sprintf('Segment %d Analysis', s), 'Color', 'w'); hold on; grid on;
    h_demo = plot3(Segments(s).Demos(1).pos(1,:), Segments(s).Demos(1).pos(2,:), Segments(s).Demos(1).pos(3,:), 'Color', [0.7 0.7 0.7]);
    for n = 2:model.nbSamples
        plot3(Segments(s).Demos(n).pos(1,:), Segments(s).Demos(n).pos(2,:), Segments(s).Demos(n).pos(3,:), 'Color', [0.7 0.7 0.7], 'HandleVisibility', 'off');
    end
    h_frames = plotFrames3D(p_task); % Full frames for individual segment plots
    h_repro = plot3(x_repro(1,:), x_repro(2,:), x_repro(3,:), 'k--', 'LineWidth', 2.5);
    legend([h_demo, h_repro, h_frames(1), h_frames(4)], 'Demos', 'TP-GMR Repro', 'Start Frame', 'End Frame', 'Location', 'best');
    title(sprintf('Segment %d TP-GMM (Gripper: %d)', s, Segments(s).GripState)); view(3);
end

%% 4. Comparative Global Visualization
figure('Name', 'Global Comparative Analysis', 'Color', 'w', 'Position', [100 100 1300 500]);

% Subplot 1: All Demonstrations
subplot(1,2,1); hold on; grid on;
title({'Original Demonstrations', 'Solid: Closed (1) | Dashed: Open (0)'});
colors = lines(model.nbSamples);
h_demo_array = [];
for n = 1:model.nbSamples
    for s = 1:totalSegments
        ls = '--'; if Segments(s).GripState == 1, ls = '-'; end
        h_line = plot3(Segments(s).Demos(n).pos(1,:), Segments(s).Demos(n).pos(2,:), Segments(s).Demos(n).pos(3,:), ...
              'Color', colors(n,:), 'LineStyle', ls, 'LineWidth', 1.2);
        if s == 1, h_demo_array = [h_demo_array, h_line]; end
    end
end
view(3); xlabel('x'); ylabel('y'); zlabel('z');
legend(h_demo_array, arrayfun(@(x) sprintf('Demo %d', x), 1:model.nbSamples, 'UniformOutput', false));

% Subplot 2: TP-GMM Concatenated Reproduction
subplot(1,2,2); hold on; grid on;
title({'TP-GMM Concatenated Reproduction', 'Colored by Segment'});
seg_colors = lines(totalSegments);
h_seg_array = [];
for s = 1:totalSegments
    ls = '--'; if Segments(s).GripState == 1, ls = '-'; end
    h_s = plot3(Learned_Segments{s}(1,:), Learned_Segments{s}(2,:), Learned_Segments{s}(3,:), ...
          'LineWidth', 3, 'LineStyle', ls, 'Color', seg_colors(s,:));
    h_seg_array = [h_seg_array, h_s];
    
    % Visualization of start/end as small points
    plot3(Segments(s).p_task(1).b(1), Segments(s).p_task(1).b(2), Segments(s).p_task(1).b(3), ...
          'go', 'MarkerFaceColor', 'g', 'MarkerSize', 4, 'HandleVisibility', 'off'); % Green point
    plot3(Segments(s).p_task(2).b(1), Segments(s).p_task(2).b(2), Segments(s).p_task(2).b(3), ...
          'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 4, 'HandleVisibility', 'off'); % Red point
end
view(3); xlabel('x'); ylabel('y'); zlabel('z');
legend(h_seg_array, arrayfun(@(x) sprintf('Seg %d', x), 1:totalSegments, 'UniformOutput', false), 'Location', 'best');