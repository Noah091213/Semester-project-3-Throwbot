function output = throwCalc(input)

targetPosition = [input(1:3)];
followTime = input(4);
frequency = input(5);
transformW2R    = [input(6:9); 
                   input(10:13); 
                   input(14:17); 
                   input(18:21)];
fileNumber = input(22);

releasePositions = [0.35, 0.40, 0.40;
                  0.50, 0.40, 0.35;
                  0.45, 0.50, 0.40;
                  0.50, 0.50, 0.40;
                  0.40, 0.40, 0.35;
                  0.35, 0.50, 0.35;
                  0.40, 0.50, 0.35];

jointAcceleration = 5;

status = 100000;
error = 9;

for i = 1:7

    releasePosition = releasePositions(i, 1:3);

    fprintf('[Matlab Info] Testing release position: [%6.3f %6.3f %6.3f ]\n', releasePosition);

    positionDifference = norm(releasePosition(1:2) - targetPosition(1:2));

    if positionDifference < 0.20
        if i ~= 7
            fprintf('[Matlab Error] Horizontal distance <20cm. Trying next release point...\n')
        end
        continue
    end

    [yaw, pitch, velocity] = trajectory(releasePosition, targetPosition);

    if isnan(yaw) || isnan(pitch) || isnan(velocity)
        fprintf('[Matlab Error] Trajectory calculation failed. Trying next release point...\n')
    end

    fprintf('[Matlab Info] Yaw, pitch, velocity: [%6.3f %6.3f %6.3f ]\n', [yaw, pitch, velocity]);

    [status, q, qd] = throwFunction(releasePosition, targetPosition, yaw, pitch, velocity, jointAcceleration, followTime, frequency, transformW2R, fileNumber);

    fprintf('[Matlab Info] Status: %.2f\n', status);

    if status < 100

        error = error * 100; 
        error = error + status; 
        
        if i == 7
            output = error;
            return
        end
        fprintf('[Matlab Error] No viable path found. Trying next release point...\n')
        continue
    end

    releaseIndex = size(q,2)-round(followTime*frequency);
    qRelease = q(:,releaseIndex);
    qdRelease = qd(:,releaseIndex);

    fprintf('[Matlab Info] \n');
    fprintf('[Matlab Info] SOLUTION FOUND!\n');
    fprintf('[Matlab Info] \n');
    fprintf('[Matlab Info] Release at q: %.2f\n', releaseIndex);
    fprintf('[Matlab Info] Release pose (deg): [%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f]\n', qRelease);
    fprintf('[Matlab Info] Release pose (rad): [%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f]\n', deg2rad(qRelease));
    fprintf('[Matlab Info] Release velocity (deg): [%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f]\n', rad2deg(qdRelease));
    fprintf('[Matlab Info] Release velocity (rad): [%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f]\n', qdRelease);


    output = [status, deg2rad(q(:,1))'];

    for j = 1:size(qd,2)
        for y = 1:6
            output(end+1) = qd(y,j);
        end
    end
    return

end

end

function [status, q, qd] = throwFunction(releasePosition, targetPosition, yaw, pitch, releaseVelocity, jointAcceleration, followTime, frequency, transformW2R, fileNumber)

%% 1. Parameters

% Initialize variables
status = 100000;
dt = 1 / frequency; % Delta time; duration of each time step (s)
q = 0;
qd = 0;

% Set joint limits
q_min = [-160 -180 -145 -100  65 -360];
q_max = [  25    0    0   90 105  360];

qStart = deg2rad((q_min + q_max) / 2);

% Apply offset for IK solution
IKoffset = [20 20 15 15 15 5];

q_min_IK = deg2rad(q_min + IKoffset);
q_max_IK = deg2rad(q_max - IKoffset);

% Apply offset for safety checks
safetyOffset = 5;

q_min = deg2rad(q_min + safetyOffset);
q_max = deg2rad(q_max - safetyOffset);

% Set TCP limits in world frame (m)
x_min = 0.00; x_max = 1.23;
y_min = 0.18; y_max = 0.64;
z_min = 0.19; z_max = 2;

% Transform desired release point to base frame
releasePos = transformPosition(releasePosition, transformW2R); 

% Extract rotation and translation part from W2R transform
R = transformW2R(1:3,1:3);
t = transformW2R(1:3,4);

% Calculate R2W transform based on W2R
R2W = [R'  -R'*t;
       0 0 0 1];


%% 2. Load and Calibrate UR5 Model

% Load UR5 robot with gravity
robot = loadrobot("universalUR5","DataFormat","row");  
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

% Rotate the model 180 deg to align with real UR5 robot
Rz180 = axang2tform([0 0 1 pi]);
robot.Base.Children{1}.Joint.setFixedTransform(Rz180);

% Create a new rigid body for the TCP offset
tcp = rigidBody('tcp_offset');

% Define the 17 cm offset along Z
offset = trvec2tform([0 0 0.17]);  % meters
setFixedTransform(tcp.Joint, offset);

% Attach it to the last link (tool0)
addBody(robot, tcp, robot.BodyNames{end});

%% 3. Compute TCP orientation

% Create a normalized vector to indicate direction of throw (World Frame)
dir_world = [cos(pitch)*(-cos(yaw)); 
             cos(pitch)*(sin(yaw)); 
             sin(pitch)];
dir_world = dir_world / norm(dir_world);

fprintf('[Matlab Info] Direction vector: [%6.3f %6.3f %6.3f ]\n', dir_world);

% Rotate dir to be in base frame
dir = R * dir_world;
dir = dir / norm(dir);

% Create X, Y, Z for TCP in World Frame
% Y-axis is opposite the direction vector
y_axis = -dir_world;

% Z-axis is pointing "down", found by projecting Y onto a vector pointing straight down
g_down = [0; 0; -1];
z_axis = g_down - (g_down' * y_axis) * y_axis;
z_axis = z_axis / norm(z_axis);

% X axis completes the right-handed frame: x = y × z
x_axis = cross(y_axis, z_axis);
x_axis = x_axis / norm(x_axis);

% Recompute Z to clean rounding errors
z_axis = cross(x_axis, y_axis);
z_axis = z_axis / norm(z_axis);

% Form rotation matrix based on axis
R_release = [x_axis, y_axis, z_axis]; % World Frame
R_release = R * R_release; % Apply rotation to Base Frame



%% 4. Compute Inverse Kinematics for release configuration

% Create a GIK object using UR5 rigidBodyTree and 3 contraint variables
gik = generalizedInverseKinematics('RigidBodyTree', robot, 'ConstraintInputs', {'jointbounds','position','orientation'});

% Create joint constraint based on joint limits with heavy offset to push solution towards center of limits
jointConst = constraintJointBounds(robot);
jointConst.Bounds = [q_min_IK', q_max_IK'];
jointConst.Weights = [500, 500, 500, 500, 500, 500]; % All joint constraint weighted heavily to enforce limits

% Create position constraint based on release position
posConst = constraintPositionTarget('tcp_offset');
posConst.TargetPosition = releasePos;
posConst.Weights = 50; % Position weighted more than orientation to push solution to correct release point

% Create orientation constraint based on TCP orientation at release point
orientConst = constraintOrientationTarget('tcp_offset');
orientConst.TargetOrientation = rotm2quat(R_release);
orientConst.Weights = 1; % Orientation weighted very low as to give IK more space for solution

% Compute Inverse Kinematics solution based on above constraints
[q_release, solutionInfo] = gik(qStart, jointConst, posConst, orientConst);

% Check if solution was found
if solutionInfo.ExitFlag <= 0
    status = 20;
    return;
end

% Ensure wrist3 does not rotate unneccessarily
if q_release(6) < -pi
    q_release(6) = q_release(6) + 2*pi;
end
if q_release(6) > pi
    q_release(6) = q_release(6) - 2*pi;
end

% Clip any joint outside limits
for i = 1:6
    if q_release(i) < q_min(i)
        q_release(i) = q_min(i);
    end
    if q_release(i) > q_max(i)
        q_release(i) = q_max(i);
    end
end

% Calculate error, i.e. how far is solution position from desired release point
[computedRPos, ~, ~] = kinUR5(q_release);
p_hom = [computedRPos(:); 1];   
computedRtcp = R2W * p_hom;  
releasePosDiff = norm(releasePosition - computedRtcp(1:3)');

% Set status based on error calculation
if releasePosDiff > 0.001
    status = status + 10000 * round(releasePosDiff*1000);
end
if releasePosDiff > 0.005
    status = 21;
    return
end

%% 5. Compute joint velocity at release point (Weighted / Soft Lock)

% Calculate Jacobian and extract velocity portion
[~,~,J] = kinUR5(q_release);
Jv = J(1:3, :); 

% Create weighted Jacobian such that wrist2 is more expensive to use
W_diag = [1, 1, 1, 1, 5, 1]; 
W_inv = diag(1 ./ W_diag);
J_weighted_inv = W_inv * Jv' / (Jv * W_inv * Jv');

% Calculate initial velocities for joints
qd_release_init = J_weighted_inv * (releaseVelocity * dir);

% Project onto direction vector to ensure trajectory and scale to ensure speed
factor = (dir' * (Jv * qd_release_init)); 
scaling = releaseVelocity / factor;      
qd_release = qd_release_init * scaling; 

% Check calculated release velocity, set status based on error
v_release = Jv * qd_release;
speed_release = norm(v_release);
speedDiff = 1 - speed_release / releaseVelocity;
if speedDiff > 0.001
    status = status + 1000 * round(speedDiff*100);
end
if speedDiff > 0.03
    status = 22;
    return
end

%% 6. Generate lead-up trajectory in joint space

% Find the highest joint velocity required at release
maxJointVelocity = max(abs(qd_release));

% Calculate global leadTime necessary to hit target velocity with given acceleration
leadTime = maxJointVelocity / jointAcceleration;

% Calculate global points needed for lead trajectory
numLeadMax = ceil(leadTime * frequency);

% Initialize matrices
q_lead = zeros(6, numLeadMax);
qd_lead = zeros(6, numLeadMax);

% Loop each joint
for i = 1:6

    % Set leadTime and points for specific joint
    leadTimeMax = abs(qd_release(i)) / jointAcceleration;
    numLead = ceil(leadTimeMax * frequency);

    % Reset qi_prev to release position
    qi_prev = q_release(i);

    % Loop through global points
    for j = 1:numLeadMax

        % Set change in velocity needed for next point
        deltaVelocity = max((1 - j / numLead), 0);

        % Calculate velocity at next point
        qdi_temp = deltaVelocity * qd_release(i);

        % Calculate joint position at next point
        qi_next = qi_prev - qdi_temp * dt;

        % Update matrices with found point (working from release towards start)
        q_lead(i, numLeadMax+1-j) = qi_next;
        qd_lead(i, numLeadMax+1-j) = qdi_temp;

        % Set qi_prev to next point
        qi_prev = qi_next;
    end
end

% Check lead-up is within joint limits
if any(any(q_lead > q_max' | q_lead < q_min'))
    status = 23;
    return
end

%% 7. Generate follow-through manually

% Compute amount of points needed to satisfy control frequencyuency and follow time (section 1)
numFollow = round(followTime * frequency);

% Preallocate matrices
q_follow = zeros(6, numFollow);
qd_follow = zeros(6, numFollow);

% Define previous joint position
q_prev = q_release(:);

% Limit enforcement check
isEnforced = 0;

% Loop for each point in follow-through trajectory
for i = 1:numFollow

    % Calculate desired velocity at current point
    decc = 1 - i/numFollow;
    v_desired = decc * releaseVelocity * dir;

    % Compute velocity Jacobian for previous point
    [~,~,J] = kinUR5(q_prev');
    Jv = J(1:3, :);

    % Compute joint velocity
    qd_temp = pinv(Jv) * v_desired;

    % Update joint position
    q_next = q_prev + qd_temp * dt;


    % Enforce joint limits
    for j = 1:6
        if q_next(j) < q_min(j)
            q_next(j) = q_min(j);
            qd_temp(j) = 0;
            isEnforced = 1;
        end
        if q_next(j) > q_max(j)
            q_next(j) = q_max(j);
            qd_temp(j) = 0;
            isEnforced = 1;
        end
    end

    % Store in matrices
    q_follow(:,i) = q_next;
    qd_follow(:,i) = qd_temp;

    q_prev = q_next;

end

% Limit enforcement check
if isEnforced == 1
    status = status + 100;
end

%% 8. Combine trajectories

q_release_col = q_release(:);
qd_release_col = qd_release(:);

q_traj = [q_lead, q_release_col, q_follow];
qd_traj = [qd_lead, qd_release_col, qd_follow];

%% 9. TCP constraint safety check

% Check that TCP is within limits at each point
for i = 1:size(q_traj,2)
    [tcp_pos, ~, ~] = kinUR5(q_traj(:,i)');
    tcp_pos = transformPosition(tcp_pos', R2W);
    if tcp_pos(1) < x_min || tcp_pos(1) > x_max || ...
       tcp_pos(2) < y_min || tcp_pos(2) > y_max || ...
       tcp_pos(3) < z_min || tcp_pos(3) > z_max
       status = 24;
    end

end


%% Output

q = rad2deg(q_traj);
qd = qd_traj;

%% Data log

data = zeros(3, 6*size(q_traj, 1));

data(1,1:3) = releasePosition';
data(1,4) = yaw;
data(1,5) = pitch;
data(1,6) = releaseVelocity;
data(1,7) = followTime;
data(1,8) = leadTimeMax;
data(1,9) = status;
data(1,10:12) = targetPosition;

for i = 1:size(q_traj,1)
    offset = (i-1) * 6;
    data(2, 1+offset : 6+offset) = q_traj(:, 1);
    data(3, 1+offset : 6+offset) = qd_traj(:, 1);
end

filename = sprintf('/DataLogs/%d.csv', fileNumber);
writematrix(data, filename);

end

%% Helper functions

function p_out = transformPosition(p_in, T)
    p_out = zeros(1, size(p_in,2));

    % Only position part for now (3D)
    if length(p_in) >= 3
        p = [p_in(1:3) 1];  % homogeneous
        res = zeros(1,4);
        for i = 1:4
            for j = 1:4
                res(i) = res(i) + T(i,j) * p(j);
            end
        end
        p_out(1:3) = res(1:3);
    end
end

function [p,R,J] = kinUR5(q)
% Generated using UR5__symkin_o_Tool.m provided by Inigo Iturrate
% q : Joint angles (6x1) [rad]
% p : TCP position (3x1) [m]
% R : TCP rotation (3x3)
% J : Jacobian     (6x6)

q1=q(:,1); q2=q(:,2); q3=q(:,3); q4=q(:,4); q5=q(:,5); q6=q(:,6);
t2=cos(q1); t3=cos(q2); t4=cos(q3); t5=cos(q4); t6=cos(q5); t7=cos(q6);
t8=sin(q1); t9=sin(q2); t10=sin(q3); t11=sin(q4); t12=sin(q5); t13=sin(q6);
t14=t3.*t4; t15=t2.*t6; t16=t3.*t10; t17=t4.*t9; t18=t2.*t12; t19=t6.*t8; t20=t9.*t10; t21=t8.*t12; t22=-t2;
t35=t2.*t3.*(1.7e+1./4.0e+1); t36=t3.*t8.*(1.7e+1./4.0e+1); t40=t2.*1.092e-1; t41=t8.*1.092e-1;
t23=-t15; t24=t8.*t20; t25=-t20; t26=t2.*t14; t27=t2.*t16; t28=t2.*t17; t29=t8.*t14; t30=t2.*t20; t31=t8.*t16;
t32=t8.*t17; t34=t20.*t22; t37=-t35; t38=t14.*(4.9e+1./1.25e+2); t39=t20.*(4.9e+1./1.25e+2); t42=t16+t17;
t56=t15.*2.523e-1; t57=t19.*2.523e-1; t33=-t29; t43=-t38; t44=t24.*(4.9e+1./1.25e+2); t45=t14+t25;
t46=t26.*(4.9e+1./1.25e+2); t47=t27.*(4.9e+1./1.25e+2); t48=t28.*(4.9e+1./1.25e+2); t49=t29.*(4.9e+1./1.25e+2);
t50=t30.*(4.9e+1./1.25e+2); t51=t31.*(4.9e+1./1.25e+2); t52=t32.*(4.9e+1./1.25e+2); t53=t5.*t42; t54=t11.*t42;
t58=t27+t28; t59=t31+t32; t63=t26+t34; t55=-t46; t60=t5.*t45; t61=t11.*t45; t64=t24+t33; t65=t5.*t58;
t66=t11.*t58; t67=t5.*t59; t68=t11.*t59; t69=t5.*t63; t70=t11.*t63; t75=t53.*9.47e-2; t72=t5.*t64;
t73=t11.*t64; t76=t61.*9.47e-2; t77=t65.*9.47e-2; t78=t66.*9.47e-2; t79=t67.*9.47e-2; t80=t68.*9.47e-2;
t83=t69.*9.47e-2; t84=t70.*9.47e-2; t88=t53+t61; t92=t65+t70; t98=-t6.*(t66-t69); t101=t12.*(t66-t69);
t102=t12.*(t54-t60).*2.523e-1; t74=-t73; t81=-t78; t82=-t80; t85=t72.*9.47e-2; t86=t73.*9.47e-2;
t90=t12.*t88; t93=t68+t72; t104=t21+t98; t106=t19+t101; t107=t12.*t92.*2.523e-1; t111=t101.*2.523e-1;
t87=-t85; t91=-t90; t95=t67+t74; t96=t6.*t93; t97=t12.*t93; t112=t37+t41+t50+t55+t57+t77+t84+t111;
t103=t18+t96; t105=t23+t97; t108=t97.*2.523e-1;
p = [t112;-t36-t40+t44-t49-t56+t79-t86+t108;t9.*(-1.7e+1./4.0e+1)-t16.*(4.9e+1./1.25e+2)-t17.*(4.9e+1./1.25e+2)+t54.*9.47e-2-t60.*9.47e-2-t90.*2.523e-1+8.916e-2];
if nargout > 1; R = reshape([-t13.*t92+t7.*t104,-t13.*t95-t7.*t103,-t13.*(t54-t60)+t6.*t7.*t88,-t7.*t92-t13.*t104,-t7.*t95+t13.*t103,-t7.*(t54-t60)-t6.*t13.*t88,t106,t105,t91],[3,3]); end
if nargout > 2; t110 = t12.*t95.*2.523e-1; J = reshape([t36+t40-t44+t49+t56-t79+t86-t108,t112,0.0,0.0,0.0,1.0,t47+t48+t81+t83+t107+t2.*t9.*(1.7e+1./4.0e+1),t51+t52+t82+t87+t110+t8.*t9.*(1.7e+1./4.0e+1),t3.*(-1.7e+1./4.0e+1)+t39+t43+t75+t76+t102,t8,t22,0.0,t47+t48+t81+t83+t107,t51+t52+t82+t87+t110,t39+t43+t75+t76+t102,t8,t22,0.0,t81+t83+t107,t82+t87+t110,t75+t76+t102,t8,t22,0.0,t21.*(-2.523e-1)+t6.*(t66-t69).*2.523e-1,t18.*2.523e-1+t96.*2.523e-1,t6.*t88.*(-2.523e-1),t92,t95,t54-t60,0.0,0.0,0.0,t106,t105,t91],[6,6]); end
end

function [yaw, pitch, velocity] = trajectory(releasePosition, targetPosition)
% TRAJECTORY Finder den minimale starthastighed og de tilhørende vinkler
% (yaw og pitch) for at ramme et mål.
%   yaw og pitch returneres nu i RADIANER.
%
%   releasePosition: [x_r, y_r, z_r] - Startposition (m)
%   targetPosition: [x_t, y_t, z_t] - Målposition (m)
    % --- Konstanter ---
    g = 9.81; % Tyngdeacceleration (m/s^2)
    minPitch_deg = 22.5; % Minimum pitch-vinkel (grader, bruges KUN til beregning)
    
    % --- Bestem Forskydning ---
    delta_r = targetPosition - releasePosition;
    
    % Horisontal afstand (i xy-planet)
    R_xy = sqrt(delta_r(1)^2 + delta_r(2)^2);
    
    % Vertikal afstand (z-akse)
    delta_z = delta_r(3); 
    
    if R_xy == 0
        fprintf('[Matlab error] Target is directly under release point\n');
        return
    end
    % --- 1. Bestem Yaw (Resultat i RADIANER) ---
    % Yaw: 0 rad i -x retning. Positiv mod +y, Negativ mod -y.
    
    theta_from_plus_x = atan2(delta_r(2), delta_r(1)); % Vinkel fra +x-aksen
    
    % Vinklen fra den NEGATIVE X-akse (wrappet til [-pi, pi])
    yaw = pi - theta_from_plus_x;
    yaw = mod(yaw + pi, 2*pi) - pi; % yaw er nu i radianer [-pi, pi]
    
    % --- 2. Optimer Pitch og Velocity (Resultat i RADIANER) ---
    
    minPitch_rad = deg2rad(minPitch_deg);
    
    % Søg efter pitch i intervallet [minPitch, 89.99 grader]
    pitch_search_rad = linspace(minPitch_rad, deg2rad(89.99), 1000); 
    
    v0_squared = zeros(size(pitch_search_rad));
    
    for i = 1:length(pitch_search_rad)
        theta = pitch_search_rad(i);
        
        % V0^2 ligningen:
        % v0^2 = (g * R_xy^2) / (2 * cos(theta)^2 * (R_xy * tan(theta) - delta_z))
        denominator = 2 * cos(theta)^2 * (R_xy * tan(theta) - delta_z);
        
        if denominator > 1e-6 
            v0_squared(i) = (g * R_xy^2) / denominator;
        else
            v0_squared(i) = Inf; 
        end
    end
    
    % Find den minimale hastighed
    [min_v0_squared, min_idx] = min(v0_squared);
    
    velocity = sqrt(min_v0_squared);
    
    % Tildel optimal pitch i radianer
    pitch = pitch_search_rad(min_idx); 
    
    % Håndter kravet om minimum pitch (nu i radianer)
    if pitch < minPitch_rad
       pitch = minPitch_rad; % Brug minPitch i radianer
       
       % Genberegn hastighed for den påkrævede minimum pitch
       denominator_min = 2 * cos(pitch)^2 * (R_xy * tan(pitch) - delta_z);
       if denominator_min > 0
           velocity = sqrt((g * R_xy^2) / denominator_min);
       else
           fprintf('[Matlab error] Cannot hit target with minimum angle\n'); 
       end
    end
end