%% MRI Coursework - Task 2
% M Paz Cardona - 01410045

clc
load('trajectory_data.mat')
load('robot.mat')

%% Task 3 i - task 1 with constant velocity

T_beat = transl(pos_beat_trajectory');  % homogeneous transform of end-effector pose
J = jacobe(rob, T_beat); % calculate jacobian
vel = [0.1 0.1 0.1 0.1 0.1 0.1]'; % given end-effector velocity
qvel = inv(J) .* vel; % obtain joints velocities

q_beat = rob.ikine(T_beat,qvel); % inverse kinematics
plot2(transl(T_beat)) % plot trajectory
rob.plot(q_beat,'movie','Task3a.mp4') % display and save animation

% End-effector position error
error = zeros(3,29); % initialise error vector
fk = rob.fkine(q_beat); % forward kinematics to retrieve end-effector pose
for n = 1:29
    error(:,n) = fk(n).t - pos_beat_trajectory(:,n); % compare fk to given trajectory
end


error=abs(error);
figure; 
subplot(4,1,1)
plot(error(1,:))
xlabel('Trajectory step')
ylabel('X-Coord Error')
subplot(4,1,2)
plot(error(2,:))
xlabel('Trajectory step')
ylabel('Y-Coord Error')
subplot(4,1,3)
plot(error(3,:))
xlabel('Trajectory step')
ylabel('Z-Coord Error')

% End-effector velocity error
fk = rob.fkine(q_beat); % forward kinematics to retrieve end-effector pose

v_vec = zeros(4,29); % initialise actual end-effector velocity vector
v_error = zeros(4,29); % initialise velocity error vector
for n = 2:29
    dx = fk(1,n).t-fk(1,n-1).t;
    dt = norm(dx)/0.1;
    v = dx/dt;
    v_vec(1:3,n) = v;
    v_vec(4,n) = norm(v(1:2,1)); % norm of velocities in xyz coordinates - overall velocity
end

for n = 1:29        
    v_error(:,n) = v_vec(:,n)-[0.1 0.1 0.1 0.1]'; % compare actual to given velocities 
end

v_error=abs(v_error);
subplot(4,1,4) 
plot(v_error(4,:)) % plot only last column which represents overall velocity
ylim([-0.01 0.11])
xlabel('Trajectory step')
ylabel('Velocity Error')
sgtitle('End-effector position and velocity error')


%% Task 3 ii
R_beat = zeros(3,3,29);
for n=1:29
     R_beat(:,:,n) = angvec2r(alpha(n),vector); % initialise rotation matrix with given angle orientation
end

TR_beat = rt2tr(R_beat, pos_beat_trajectory); % obtain homogeneous transformation for rotation + translation
q_beat = rob.ikine(TR_beat, qvel'); % inverse kinematics to get joints coordinates
figure;
plot2(transl(TR_beat)) % plot trajectory
rob.plot(q_beat,'movie','Task3b.mp4') % display and save animation

% End-effector position error
error = zeros(3,29); % initialise error vector
fk = rob.fkine(q_beat);  % forward kinematics to retrieve end-effector pose
for n = 1:29
    error(:,n) = fk(n).t - pos_beat_trajectory(:,n); % compare fk to given trajectory
end


error=abs(error);
figure; 
subplot(4,1,1)
plot(error(1,:))
xlabel('Trajectory step')
ylabel('X-Coord Error')
subplot(4,1,2)
plot(error(2,:))
xlabel('Trajectory step')
ylabel('Y-Coord Error')
subplot(4,1,3)
plot(error(3,:))
xlabel('Trajectory step')
ylabel('Z-Coord Error')


% End-effector velocity error
fk = rob.fkine(q_beat); % forward kinematics to retrieve end-effector pose

v_vec = zeros(4,29); % initialise actual end-effector velocity error
v_error = zeros(4,29); % initialise velocity error vector
for n = 2:29
    dx = fk(1,n).t-fk(1,n-1).t; 
    dt = norm(dx)/0.1;
    v = dx/dt;
    v_vec(1:3,n) = v;
    v_vec(4,n) = norm(v(1:2,1)); % norm of velocities in xyz coordinates - overall velocity
end

for n = 1:29        
    v_error(:,n) = v_vec(:,n)-[0.1 0.1 0.1 0.1]';  % compare actual to given velocities 
end

v_error=abs(v_error);
subplot(4,1,4) 
plot(v_error(4,:)) % plot only last column which represents overall velocity
ylim([-0.01 0.11])
xlabel('Trajectory step')
ylabel('Velocity Error')
sgtitle('End-effector position and velocity error')

% End-effector orientation error
orient_error = zeros(3,29); % initialise orientation error
true_angle = alpha'*vector; % given angles
calc_angle=zeros(29,3); % initialise actual angles vector


for n = 1:29
    fkR = tr2rt(fk(n));  % rotation matrix from fkine homogeneous transform
    calc_angle(n,:) = rotm2eul(fkR);  % Euler angles from rotation matrix  
end  

calc_angle = fliplr(calc_angle);  % zyx to xyz
calc_angle(:,3)=(360+(calc_angle(:,3)*180/pi))*pi/180; % put in right quadrant
for n=1:29
    orient_error(:,n) = true_angle(n,:)-calc_angle(n,:); % error between actual and given orientation
end

orient_error=abs(orient_error);
figure;
subplot(3,1,1)
plot(orient_error(1,:))
xlabel('Trajectory step')
ylabel('X-angle Error')
subplot(3,1,2)
plot(orient_error(2,:))
xlabel('Trajectory step')
ylabel('Y-angle Error')
subplot(3,1,3)
plot(orient_error(3,:))
xlabel('Trajectory step')
ylabel('Z-angle Error')
sgtitle('End-effector orientation error')
