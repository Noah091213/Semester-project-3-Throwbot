%% UR5 Throwing Trajectory Planner (Joint-Space) with Perpendicular Gripper and Smooth Follow-Through

clc; clear; close all;

%% -------------------- 1. Parameters --------------------
releasePos = [0.5, 0.5, 0.5];    % Release position (x,y,z) in meters
yaw = pi/4;                      % Yaw angle of throw (rad)
pitch = pi/6;                    % Pitch angle of throw (rad)
v_release = 1.0;                 % Release speed (m/s)
t_release = 5.0;                 % Nominal time to reach release (s)
numPoints = 500;                 % Points for main trajectory
fol_time = 1.0;                  % Duration of follow-through (s)
numFollow = 100;                 % Points for follow-through

q_start = deg2rad([-100 -100 -50 30 70 -330]);  % Initial joint configuration (rad)

% Joint limits
q_min = deg2rad([-160 -180 -145 -100 65 -360]);
q_max = deg2rad([ 25 0 0 90 105 360]);

%% -------------------- 2. Load UR5 Model --------------------
robot = loadrobot("universalUR5","DataFormat","row");  
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

%% -------------------- 3. Compute end-effector orientation (perpendicular gripper) --------------------
dir = [cos(pitch)*cos(yaw), cos(pitch)*sin(yaw), sin(pitch)];  % Throw direction
dir = dir(:)/norm(dir);

% Reference vector to define perpendicular axes
refVec = [0;0;1];
if abs(dot(dir, refVec)) > 0.99
    refVec = [0;1;0];  % avoid singularity if aligned
end

% Construct orthonormal frame
x_axis = cross(refVec, dir); x_axis = x_axis/norm(x_axis); % tool X perpendicular to throw
y_axis = cross(dir, x_axis); y_axis = y_axis/norm(y_axis); % tool Y
z_axis = dir;                                               % tool Z along throw

R_release = [x_axis, y_axis, z_axis];                      % Rotation matrix
T_release = trvec2tform(releasePos) * rotm2tform(R_release);

%% -------------------- 4. Solve IK for release configuration --------------------
gik = generalizedInverseKinematics('RigidBodyTree', robot, ...
    'ConstraintInputs', {'position','orientation','jointbounds'});

posConst = constraintPositionTarget('tool0');
posConst.TargetPosition = releasePos;

orientConst = constraintOrientationTarget('tool0');
orientConst.TargetOrientation = rotm2quat(R_release);

jointConst = constraintJointBounds(robot);
jointConst.Bounds = [q_min', q_max'];

[q_release, solutionInfo] = gik(q_start, posConst, orientConst, jointConst);
if solutionInfo.ExitFlag <= 0
    error('No feasible IK solution found. Adjust release pose.');
end

%% -------------------- 5. Compute joint velocity for release along throw --------------------
J = geometricJacobian(robot, q_release, 'tool0');  % 6x6
Jv = J(1:3,:);                                      % linear part

% initial joint velocity
qd_release_init = pinv(Jv) * (v_release * dir);    

% project to ensure exact EE velocity along desired direction
qd_release = pinv(Jv) * (dir * (dir' * (Jv*qd_release_init)));

%% -------------------- 6. Generate main trajectory in joint space --------------------
t_main = linspace(0, t_release, numPoints);
[q_traj, qd_traj, qdd_traj] = cubicpolytraj([q_start; q_release]', [0 t_release], t_main, ...
                                             'VelocityBoundary',[zeros(6,1) qd_release]);

%% -------------------- 7. Generate follow-through while maintaining orientation --------------------
t_follow = linspace(0, fol_time, numFollow);
q_follow = zeros(6, numFollow);
qd_follow = zeros(6, numFollow);
qdd_follow = zeros(6, numFollow);

q_prev = q_release(:);

for i = 1:numFollow
    alpha = 1 - i/numFollow;                  % deceleration factor
    v_desired = alpha * v_release * dir;      % desired EE velocity
    
    J = geometricJacobian(robot, q_prev', 'tool0'); % 6x6
    Jv = J(1:3,:);                                  % linear part
    
    % Compute joint velocity
    qd_temp = pinv(Jv) * v_desired;
    
    % Incrementally update joint position
    dt = t_follow(2)-t_follow(1);
    q_next = q_prev + qd_temp*dt;
    
    % Enforce joint limits
    q_next = min(max(q_next, q_min'), q_max');
    
    % Store
    q_follow(:,i) = q_next;
    qd_follow(:,i) = qd_temp;
    if i>1
        qdd_follow(:,i) = (qd_follow(:,i)-qd_follow(:,i-1))/dt;
    end
    
    q_prev = q_next;
end

%% -------------------- 8. Combine trajectories --------------------
q_traj = [q_traj, q_follow];
qd_traj = [qd_traj, qd_follow];
qdd_traj = [qdd_traj, qdd_follow];
t = [t_main, t_release + t_follow];

%% -------------------- 9. Visualize --------------------
figure;
show(robot, q_start, 'PreservePlot', false); hold on; 
plot3(releasePos(1), releasePos(2), releasePos(3), 'ro','MarkerSize',10,'LineWidth',2); 

for i = 1:5:length(q_traj)
    show(robot, q_traj(:,i)', 'PreservePlot', false);
    drawnow;
end

title('UR5 Throwing Trajectory with Perpendicular Gripper and Follow-Through');
xlabel('X'); ylabel('Y'); zlabel('Z'); grid on; axis equal;

%% -------------------- 10. End-effector velocity and direction --------------------
[nJoints, nPoints] = size(q_traj);
eeSpeed = zeros(1,nPoints);
eeDir = zeros(3,nPoints);

for i=1:nPoints
    Jv = geometricJacobian(robot, q_traj(:,i)', 'tool0'); 
    Jv = Jv(1:3,:);
    v = Jv * qd_traj(:,i);
    eeSpeed(i) = norm(v);
    if eeSpeed(i)>0
        eeDir(:,i) = v/eeSpeed(i);
    else
        eeDir(:,i) = [0;0;0];
    end
end

[~, releaseIdx] = min(abs(t - t_release));

figure;
plot(t, eeSpeed, 'LineWidth', 2); hold on;
yline(v_release, 'r--','Target release speed','LineWidth',1.5);
plot(t(releaseIdx), eeSpeed(releaseIdx), 'ko','MarkerSize',8,'MarkerFaceColor','k');
text(t(releaseIdx), eeSpeed(releaseIdx)+0.05,'Release point','HorizontalAlignment','center');
grid on; xlabel('Time [s]'); ylabel('End-effector speed [m/s]');
title('End-effector Cartesian Speed Over Trajectory');
legend('EE speed','Target release speed','Release point');

disp('EE direction at release (unit vector):');
disp(eeDir(:,releaseIdx));
disp('Desired throw direction:');
disp(dir);

%% -------------------- 11. Release joint configuration --------------------
q_release_deg = rad2deg(q_release);
disp('Release joint configuration (deg):');

disp(q_release_deg);

%% -------------------- 12. Export trajectory ---------------------
writematrix([tvec(:), q_traj],'path.csv');
fprintf('Prajectory exported to path.csv\n');
