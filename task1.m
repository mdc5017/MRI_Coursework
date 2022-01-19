%% MRI Coursework - Task 1
% M Paz Cardona - 01410045

clc
load('trajectory_data.mat')

%% DH Table

% specify links lengths
L1 = 0.1;
L2 = 0.4;
L3 = 0.8;
L4 = 0.8;
L5 = 0.4;
L6 = 0.3; 

% define links and serial link robot
L(1) = Link('revolute', 'd', L1,'a', 0, 'alpha',0,'modified');
L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha',-pi/2, 'offset',-pi/2, 'modified');
L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha',0, 'modified');
L(4) = Link('revolute', 'd', L4,'a', 0, 'alpha', -pi/2, 'modified');
L(5) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2, 'modified');
L(6) = Link('revolute', 'd', 0, 'a', -L5, 'alpha', -pi/2,'offset',pi,'modified');

% joint limits
% L(2).qlim = [0 pi];
% L(3).qlim = [-pi pi/2];
% L(5).qlim = [-pi 0];
% L(6).qlim = [-145/180 145/180]*pi;
rob = SerialLink(L(1,1:6), 'name','robot');
rob.tool = transl(L6,0, 0);

% plot zero configuration
%plot(rob,zeros(1,6),'workspace',[0, 2.7, 0, 2.7, 0, 2.7])
rob.teach
save('robot.mat','rob');


%% Linear Trajectory

T = transl(pos_static_trajectory');  % homogeneous transform of end-effector pose 
q = rob.ikine(T); % inverse kinematics to obtain joints coordinates 
plot2(transl(T)) % plot trajectory
rob.plot(q,'movie','Task1.mp4') % display and save animation
 

%% End-Effector Position Error 
error = zeros(3,29); % initialise error vector
fk = rob.fkine(q); % forward kinematics to retrieve end-effector pose
for n = 1:29
    error(:,n) = fk(n).t - pos_static_trajectory(:,n); % compare fk to given trajectory
end

% plot end-effector position error
error=abs(error);
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

