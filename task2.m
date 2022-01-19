%% MRI Coursework - Task 2
% M Paz Cardona - 01410045

clc
load('trajectory_data.mat')
load('robot.mat')


%%  Linear trajectory perpendicular to sphere

R = zeros(3,3,29);
for n=1:29
     R(:,:,n) = angvec2r(alpha(n),vector); % initialise rotation matrix with given angle orientation
end

TR = rt2tr(R,pos_static_trajectory); % obtain homogeneous transformation for rotation + translation
q = rob.ikine(TR ); % inverse kinematics to get joints coordinates
plot2(transl(TR)) % plot trajectory
rob.plot(q,'movie','Task2.mp4') % display and save animation


%% Error

% Position error
error = zeros(3,29); % initialise error vector
fk = rob.fkine(q); % forward kinematics to retrieve end-effector pose
for n = 1:29
    error(:,n) = fk(n).t - pos_static_trajectory(:,n); % compare fk to given trajectory
end

% Orientation error
orient_error = zeros(3,29); % initialise orientation error
true_angle = alpha'*vector; % given angles
calc_angle=zeros(29,3); % initialise actual angles vector

for n = 1:29
    fkR = tr2rt(fk(n)); % rotation matrix from fkine homogeneous transform
    calc_angle(n,:) = rotm2eul(fkR);  % Euler angles from rotation matrix  
end
calc_angle = fliplr(calc_angle); % zyx to xyz
calc_angle(:,3)=(360+(calc_angle(:,3)*180/pi))*pi/180; % put in right quadrant
for n=1:29
    orient_error(:,n) = true_angle(n,:)-calc_angle(n,:); % error between actual and given orientation
end


% plot end-effector pose error
orient_error=abs(orient_error);
error = abs(error);
figure; 
subplot(3,1,1)
plot(error(1,:))
xlabel('Trajectory step')
ylabel('X-Coord Error')
subplot(3,1,2)
plot(error(2,:))
xlabel('Trajectory step')
ylabel('Y-Coord Error')
subplot(3,1,3)
plot(error(3,:))
xlabel('Trajectory step')
ylabel('Z-Coord Error')
sgtitle('End-effector position error')

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

