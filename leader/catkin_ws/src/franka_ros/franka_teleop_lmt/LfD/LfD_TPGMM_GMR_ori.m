clear all;
close all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RMIL @TU München 2025
% Learning from demonstration using TP-GMM (Orientation Enabled)
% GMM/GMR Based Reproduction Using P(x, q, v, w)
% Conditioning on (Position, Quaternion) to predict (LinVel, AngVel)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Parameters
model.nbStates = 6; 
model.nbVar = 13;   % [Pos(3), Quat(4), LinVel(3), AngVel(3)]
model.nbD = 100;
model.nbSamples = 3;
model.dt = 0.1;
reg = 1e-4; 
model.reg = reg;
dataset_path = '/home/student1/catkin_ws/src/franka_ros/franka_teleop_lmt/TeleopData/';

%% Load Demonstrations
for n = 1:model.nbSamples
    fileName = [dataset_path 'follower_' num2str(n) '.txt'];
    
    % ASSUMPTION: loadDemos now returns [7 x T] matrix -> [Pos(3); Quat(4)]
    % If your file is different, adjust indices below.
    raw_data = loadDemos(fileName)'; 
    
    % 1. Extract Raw Data
    raw_pos = raw_data(1:3, :);
    raw_quat = raw_data(4:7, :); % Format: [w, x, y, z]
    
    % 2. Interpolate Position (Spline)
    pos = spline(1:size(raw_pos,2), raw_pos, linspace(1, size(raw_pos,2), model.nbD));
    
    % 3. Interpolate Quaternion (Linear + Normalization)
    % Standard spline creates invalid quaternions, so we interp and normalize
    quat = interp1(1:size(raw_quat,2), raw_quat', linspace(1, size(raw_quat,2), model.nbD))';
    quat = quat ./ vecnorm(quat); % Force unit norm
    
    % 4. Compute Linear Velocity
    lin_vel = [diff(pos,1,2), zeros(3,1)] ./ model.dt;
    
    % 5. Compute Angular Velocity
    % Formula: w = 2 * dq * q_conj
    dquat = [diff(quat,1,2), zeros(4,1)] ./ model.dt; 
    ang_vel = zeros(3, model.nbD);
    
    for t = 1:model.nbD
        % q_dot * q_inv
        % Note: If you lack Robotics Toolbox, you need a custom quatmultiply
        temp_q = quatmultiply(dquat(:,t)', quatconj(quat(:,t)'));
        ang_vel(:,t) = 2 * temp_q(2:4)'; % Extract vector part [x,y,z]
    end

    % 6. Stack Data [Pos; Quat; LinVel; AngVel]
    demos(n).pos = pos;
    demos(n).Data1 = [pos; quat; lin_vel; ang_vel];
end

%% Frames (Red is start, Green/Custom is end)
% Format: [x, y, z, roll, pitch, yaw]
red_frame = [0.41  0.15 0.40 0 0  1.5708];  
green_frame = [0.67 -0.32 0.40 0 0 -0.8708]; 
custom_frame = [0.55 0.25 0.40 0 0 0.5];

% Define Frames for Training Demos
for n = 1:model.nbSamples
    % --- Start Frame (Red) ---
    demos(n).p(1).A = eul2rotm(red_frame(4:6), 'XYZ');
    demos(n).p(1).b = red_frame(1:3)';
    demos(n).p(1).color = [1.0, 0.0, 0.0];
    
    % --- End Frame ---
    % Demo 1 goes to Green, others go to Custom (example logic)
    if n == 1
        target_pose = green_frame;
        col = [0.0, 1.0, 0.0];
    else
        target_pose = custom_frame;
        col = [1.0, 0.5, 0.0];
    end
    
    demos(n).p(2).A = eul2rotm(target_pose(4:6), 'XYZ');
    demos(n).p(2).b = target_pose(1:3)';
    demos(n).p(2).color = col;
end

%% Define Frames for Novel Test (Reproduction)
blue_frame = [0.5, -0.37, 0.40, 0, 0, -pi/3]; 

% Start (Same as training)
p_new(1).A = eul2rotm(red_frame(4:6), 'XYZ');
p_new(1).b = red_frame(1:3)';
p_new(1).color = [1.0, 1.0, 0.0];

% End (Novel Blue Target)
p_new(2).A = eul2rotm(blue_frame(4:6), 'XYZ');
p_new(2).b = blue_frame(1:3)';
p_new(2).color = [0.0, 0.0, 1.0];

%% Plot Demonstrations (Position Only)
figure;
hold on; grid on;
for n = 1:model.nbSamples
    plot3(demos(n).pos(1,:), demos(n).pos(2,:), demos(n).pos(3,:), 'LineWidth', 2.0);
    plotFrames3D(demos(n).p);
end
xlabel('x'); ylabel('y'); zlabel('z');
title('Demonstrations (Position)');

%% Transform Data to Local Coordinates (13D)
model.nbFrames = 2;
TPGMM_Data = zeros(model.nbVar, model.nbFrames, model.nbSamples * model.nbD);

for n = 1:model.nbSamples
    for m = 1:model.nbFrames
        for t = 1:model.nbD
            x_global = demos(n).Data1(:,t);
            
            p_g = x_global(1:3);   % Global Pos
            q_g = x_global(4:7);   % Global Quat
            v_g = x_global(8:10);  % Global Lin Vel
            w_g = x_global(11:13); % Global Ang Vel
            
            % Frame Parameters
            R = demos(n).p(m).A;     % Rotation
            b = demos(n).p(m).b;     % Translation
            q_f = rotm2quat(R);      % Frame Quaternion
            
            % --- Construct Quaternion Transformation Matrix ---
            % Logic: q_local = q_frame_inv * q_global
            % We create a matrix Q_mat such that Q_mat * q_global = q_frame_inv * q_global
            q_inv = quatconj(q_f);
            w=q_inv(1); x=q_inv(2); y=q_inv(3); z=q_inv(4);
            % Left-multiplication matrix for quaternion
            Q_mat = [w -x -y -z; 
                     x  w -z  y; 
                     y  z  w -x; 
                     z -y  x  w];
            
            % --- Build 13x13 Transformation Matrix (A_big) ---
            % Since the original code uses A' * x, we assume A_big' performs the transform.
            % So A_big must contain the TRANSPOSE of the rotation blocks.
            
            % Block 1: Position (R)
            % Block 2: Quaternion (Q_mat') -> So A'*x becomes Q_mat * q
            % Block 3: Lin Vel (R)
            % Block 4: Ang Vel (R)
            A_big = blkdiag(R, Q_mat', R, R);
            
            % Bias vector (Only position is subtracted)
            b_big = [b; 0;0;0;0; 0;0;0; 0;0;0];
            
            % Transform: x_local = A_big' * (x_global - b_big)
            TPGMM_Data(:,m,(n-1)*model.nbD + t) = A_big' * (x_global - b_big);
        end
    end
end

%% Train TP-GMM
model = trainTPGMM_tensor(TPGMM_Data, model);

%% Visualize GMM States (Position Only)
for m = 1:model.nbFrames
    figure; hold on; grid on; title(['Frame ' num2str(m) ' Perspective']);
    for n = 1:model.nbSamples
        % Extract Local Data for plotting
        x_local = TPGMM_Data(:,m,(n-1)*model.nbD+1:n*model.nbD);
        plot3(x_local(1,:), x_local(2,:), x_local(3,:), 'LineWidth', 1.0);
    end
    % Plot Gaussians (Only first 3 dimensions = Position)
    GMM3D_plot(squeeze(model.Mu(1:3,m,:)), squeeze(model.Sigma(1:3,1:3,m,:)), 1);
    xlabel('x'); ylabel('y'); zlabel('z');
    view(3);
end

%% TP-GMR Rollout (Conditioning on Pos+Quat to get Vel+AngVel)
DataIn = zeros(model.nbVar, model.nbD);
DataIn(:,1) = demos(1).Data1(:,1); % Initialize with Demo 1 start state

for t = 2:model.nbD
    h = zeros(model.nbStates, 1);
    mu_out_all = zeros(6, model.nbStates); % Output is 6D (LinVel + AngVel)
    sigma_out_all = zeros(6,6, model.nbStates);
    
    % Current Input State: Position (1:3) + Quaternion (4:7)
    curr_x = DataIn(1:7, t-1);
    
    for i = 1:model.nbStates
        Sigma_sum = zeros(13);
        Mu_sum = zeros(13,1);
        
        % Product of Gaussians (Fusion of Frames)
        for m = 1:model.nbFrames
            R = p_new(m).A;
            b = p_new(m).b;
            q_f = rotm2quat(R);
            
            q_inv = quatconj(q_f);
            w=q_inv(1); x=q_inv(2); y=q_inv(3); z=q_inv(4);
            Q_mat = [w -x -y -z; x  w -z  y; y  z  w -x; z -y  x  w];
            
            % Reconstruct A_big for the NEW frames
            A_aug = blkdiag(R, Q_mat', R, R);
            b_aug = [b; zeros(10,1)];
            
            Mu_local = model.Mu(:,m,i);
            Sigma_local = model.Sigma(:,:,m,i);
            
            % Project to Global
            Mu_global = A_aug * Mu_local + b_aug;
            Sigma_global = A_aug * Sigma_local * A_aug';
            
            % Fusion
            Sigma_sum = Sigma_sum + inv(Sigma_global + reg*eye(13));
            Mu_sum = Mu_sum + inv(Sigma_global + reg*eye(13)) * Mu_global;
        end
        
        Sigma_joint = inv(Sigma_sum + reg*eye(13));
        Mu_joint = Sigma_joint * Mu_sum;
        
        % --- GMR Operation ---
        % Input Indices: 1:7 (Pos, Quat)
        % Output Indices: 8:13 (LinVel, AngVel)
        
        mu_in = Mu_joint(1:7);
        mu_out = Mu_joint(8:13);
        
        sigma_in = Sigma_joint(1:7, 1:7);
        sigma_io = Sigma_joint(8:13, 1:7);
        
        % Regularization for stability
        sigma_in = (sigma_in + sigma_in') / 2 + eye(7)*reg;
        
        % Conditional Mean: E[v,w | x,q]
        mu_cond = mu_out + sigma_io / sigma_in * (curr_x - mu_in);
        
        % Compute Weight h(i)
        h(i) = model.Priors(i) * mvnpdf(curr_x, mu_in, sigma_in);
        mu_out_all(:,i) = mu_cond;
    end
    
    % Normalize weights
    if sum(h) < 1e-8 || any(isnan(h)), h = ones(model.nbStates,1); end
    h = h / sum(h);
    
    % Weighted Sum of Velocities
    vel_cmd = sum(mu_out_all .* h', 2); % [LinVel(3); AngVel(3)]
    
    % --- Integration ---
    % 1. Position Integration
    DataIn(1:3, t) = DataIn(1:3, t-1) + vel_cmd(1:3) * model.dt;
    
    % 2. Quaternion Integration
    curr_q = DataIn(4:7, t-1);
    omega = vel_cmd(4:6);
    
    % dq = 0.5 * omega * q
    % Construct pure quaternion from omega: [0, wx, wy, wz]
    omega_quat = [0; omega];
    dq = 0.5 * quatmultiply(omega_quat', curr_q')';
    
    next_q = curr_q + dq * model.dt;
    DataIn(4:7, t) = next_q / norm(next_q); % Normalize!
    
    % Store Velocities
    DataIn(8:13, t) = vel_cmd;
end

x_repro = DataIn(1:3, :); % Extract position for plotting/saving

%% Plot Result
figure;
hold on; grid on;
for n = 1:model.nbSamples
    plot3(demos(n).pos(1,:), demos(n).pos(2,:), demos(n).pos(3,:), 'LineWidth', 2.0);
    plotFrames3D(demos(n).p);
end
plot3(x_repro(1,:), x_repro(2,:), x_repro(3,:), 'k--', 'LineWidth', 2.0);
h_frames_new = plotFrames3D(p_new);
xlabel('x'); ylabel('y'); zlabel('z');
title('TP-GMR Reproduction (Pos+Orient) in New Frame');
legend('Demo 1', 'Demo 2', 'Demo 3', 'Reproduction');
view(3);

%% Save Learned Trajectory
% Note: We save Position(3) + Quaternion(4) = 7 columns
final_traj = DataIn(1:7, :)'; 
csvwrite([dataset_path 'learned.csv'], final_traj);
disp('Saved learned trajectory (Pos + Quat) to learned.csv');