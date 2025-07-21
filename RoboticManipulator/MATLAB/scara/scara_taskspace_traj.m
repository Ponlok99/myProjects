% Author: HOM Ponlok
% Created on: Mar 03, 2023
% scara_3DoF (3DoF)
%%
clear 
close all
clc
%% Using rottraj and Cartesian trajectory
% Define waypoints
waypoints = [
    0.5    0.5    0.5
   -0.4    0.0000    0.4
    0.2    0.4    0.2
    ];

waypointVels = 0.0 * [0   0   0
                      0   1   0
                      0   0   0];

waypointAccels = 0.2 * [0   0   0
                        0   1   0
                        0   0   0];

waypointJerks = 0.0 * [0   0   0
                       0   1   0
                       0   0   0];

tpts = 0:size(waypoints,2) - 1;
numSample = size(waypoints,2) * 51;
tvec = linspace(0,size(waypoints,2) - 1, numSample);

%% import real robot
scara = importrobot('scara.urdf', 'DataFormat', 'row');
scara.Gravity = [0 0 -9.81];
show(scara,scara.homeConfiguration,'Frames','off','PreservePlot',false,'FastUpdate',true);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
%%
% Define mini-invisible-one model (dh-parameters) (meters)
dh = struct(...
    'a', [0.425 0.345 0], ...
    'd', [0.44 0 0], ...
    'alpha', [0 pi 0]);

% [q, qd, qdd, ~] = quinticpolytraj(waypoints, tpts, tvec, ... 
%             'VelocityBoundaryCondition',waypointVels, ...
%             'AccelerationBoundaryCondition',waypointAccels);

% [p, pd, pdd] = minsnappolytraj(waypoints,tpts, numSample,...
%     'VelocityBoundaryCondition', waypointVels, ...
%     'AccelerationBoundaryCondition', waypointAccels, ...
%     'JerkBoundaryCondition', waypointJerks);

[p, pd, pdd] = minjerkpolytraj(waypoints,tpts, numSample,...
    'VelocityBoundaryCondition', waypointVels, ...
    'AccelerationBoundaryCondition', waypointAccels, ...
    'JerkBoundaryCondition', waypointJerks);

qInterp = zeros(3, numSample);
qdInterp = zeros(3, numSample);
qddInterp = zeros(3, numSample);
jointTorq = zeros(3, numSample);
       
for i=1:numSample
    T = trvec2tform(p(:,i)');
    q = scara_ikine_RRP(dh, T);
    qInterp(:, i) = q;
    jacobian = geometricJacobian(scara, q', 'tool0');
    v = [zeros(3,1); pd(:,i)];
    a = [zeros(3,1); pdd(:,i)];
    qdInterp(:,i) = jacobian \ v;
    qddInterp(:,i) = jacobian \ a;
    jointTorq(:,i) = inverseDynamics(scara, q', qdInterp(:,i)', qddInterp(:,i)');
end
figure(1)
plot3(waypoints(1,:), waypoints(2,:), waypoints(3,:), 'ro', 'MarkerSize', 5); % Original end-points
hold on
plot3(p(1,:), p(2,:), p(3,:),'Color',[0 0 1], 'LineWidth',2)
title('end-effector path in space (task space)')   

for i=1:length(tvec)
    show(scara, qInterp(:,i)', 'Frames','on', 'PreservePlot', false,'FastUpdate',true);   
    drawnow
end

figure
plot(tvec, qInterp)
grid on
title('Interpolated Joint')
legend('${q}_1$', '${q}_2$', '${q}_3$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');
xlabel('time (sec)','FontSize',13,...
       'FontWeight','bold', 'Interpreter', 'latex')
ylabel('${\theta} (rad)$','FontSize',13,...
       'FontWeight','bold', 'Interpreter', 'latex')
figure
plot(tvec, qdInterp)
grid on
title('Joint Velocities')
legend('$\dot{q}_1$', '$\dot{q}_2$', '$\dot{q}_3$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');
xlabel('time (sec)','FontSize',13,...
       'FontWeight','bold', 'Interpreter', 'latex')
ylabel('$\dot{\theta} (rad/s)$','FontSize',13,...
       'FontWeight','bold', 'Interpreter', 'latex')

figure
plot(tvec, qddInterp)
grid on
title('Joint Acceleration')
legend('$\ddot{q}_1$', '$\ddot{q}_2$', '$\ddot{q}_3$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');
xlabel('time (sec)','FontSize',13,...
       'FontWeight','bold', 'Interpreter', 'latex')
ylabel('$\ddot{\theta} (rad/s^2)$','FontSize',13,...
       'FontWeight','bold', 'Interpreter', 'latex')

figure
plot(tvec, jointTorq)
grid on
title('joint Torque')
legend('${\tau}_1$', '${\tau}_2$', '${\tau}_3$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');
xlabel('time (sec)','FontSize',13,...
       'FontWeight','bold', 'Interpreter', 'latex')
ylabel('${\tau}$','FontSize',13,...
       'FontWeight','bold', 'Interpreter', 'latex')
