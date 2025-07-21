clear
close all
clc
%% Load robot URDF
Robot = importrobot('meca500r3.urdf', 'DataFormat', 'row');
Robot.Gravity =  [0 0 -9.81];
show(Robot, Robot.homeConfiguration, 'Frames', 'off', 'PreservePlot', false,'FastUpdate',true);
% xlim([-0.4 0.4]), ylim([-0.4 0.4]), zlim([0 0.4])
drawnow
% 
% T = getTransform(Robot, Robot.homeConfiguration, 'tool0')

aik = analyticalInverseKinematics(Robot);
showdetails(aik)
% aik.KinematicGroup = opts(2).KinematicGroup;
% generateIKFunction(aik,'ikine_myRobot');

%% Using rottraj and Cartesian trajectory
% Define waypoints
wpts = [
    0.1451    0.1451    0.1451
   -0.1851    0.0000    0.1851
    0.1000    0.2000    0.1000
    ];
orientations = [0         0         0
                pi/2      pi/6      pi/1.5
                0         0         0];

waypointVels = 0.0 * [0   0   0
                      0   1   0
                      0   0   0];

waypointAccels = 0.2 * [0   0   0
                        0   1   0
                        0   0   0];

waypointJerks = 0.0 * [0   0   0
                       0   1   0
                       0   0   0];

% Define trajectory parameters
numWpts = size(wpts,2);
numSample = 51;
tvec = linspace(0, 1, numSample); % Time vector
waypointTimes = linspace(0, numWpts-1, numWpts);

%%
% Trajectory following loop
%% Initialize arrays
qInterp = zeros(6, numSample);
qdInterp = zeros(6, numSample);
qddInterp = zeros(6, numSample);
jointTorq = zeros(6, numSample);

singularityThreshold = 0.00192;

for w = 1:numWpts-1
    % Get the initial and final rotations and times for the segment
    R0 = eul2quat(orientations(:,w)');
    Rf = eul2quat(orientations(:,w+1)');
    % timeInterval = waypointTimes(w:w+1);

    % [p,pd,pdd] = quinticpolytraj(wpts(:,w:w+1),[0 1],tvec,...
    %     'VelocityBoundaryCondition', waypointVels(:,w:w+1), ...
    %     'AccelerationBoundaryCondition', waypointAccels(:,w:w+1));
    [p,pd,pdd] = minjerkpolytraj(wpts(:,w:w+1),[0 1],numSample,...
        'VelocityBoundaryCondition', waypointVels(:,w:w+1), ...
        'AccelerationBoundaryCondition', waypointAccels(:,w:w+1), ...
        'JerkBoundaryCondition', waypointJerks(:,w:w+1));    
    % [p,pd,pdd] = minsnappolytraj(wpts(:,w:w+1),[0 1],numSample,...
    %     'VelocityBoundaryCondition', waypointVels(:,w:w+1), ...
    %     'AccelerationBoundaryCondition', waypointAccels(:,w:w+1));
    
    % Find the quaternions from trajectory generation
    % [s, sd, sdd] = minsnappolytraj([0 1], [0 1], numSample);
    [s, sd, sdd] = minjerkpolytraj([0 1], [0 1], numSample);
    
    % [s, sd, sdd] = quinticpolytraj([0 1], [0 1], tvec);
    [R, omega, alpha] = rottraj(R0, Rf, [0 1], tvec, 'TimeScaling', [s; sd; sdd]);    

    for i = 1:numSample
        tfInterp = trvec2tform(p(:,i)') * quat2tform(R(:,i)');
        q0 = ikine_myRobot(tfInterp, true, true);
        jacobian = geometricJacobian(Robot, q0(1,:), 'tool0');
        v = [omega(:,i); pd(:,i)];
        a = [alpha(:,i); pdd(:,i)];
        qdInterp(:,i) = jacobian \ [omega(:,i); pd(:,i)];
        qddInterp(:,i) = jacobian \ [alpha(:,i); pdd(:,i)];
        jointTorq(:,i) = inverseDynamics(Robot, q0(1,:), qdInterp(:,i)', qddInterp(:,i)');
        % if abs(det(jacobian)) < singularityThreshold
        %     i
        %     det(jacobian)
        % else
        % end
        qInterp(:, i) = q0(1,:);
        % Visualization
        show(Robot, qInterp(:, i)', 'Frames', 'off', 'PreservePlot', false,'FastUpdate',true);
        xlim([-0.4 0.4]), ylim([-0.4 0.4]), zlim([0 0.4])
        drawnow
        hold on 
        if i == 1
            plot3(wpts(1,:),wpts(2,:),wpts(3,:),'ro','LineWidth',2);
            xlim([-0.4 0.4]), ylim([-0.4 0.4]), zlim([0 0.4])
            grid on
            hold on
        end
        pos = tform2trvec(tfInterp);
        plot3(pos(1), pos(2), pos(3), 'b.-', 'LineWidth', 4, 'MarkerSize', 3, 'MarkerFaceColor', 'r');
        grid on
    end
end

%% Plot results
figure
plot(tvec, qInterp)
grid on
title('Interpolated Joint')
legend('${q}_1$', '${q}_2$', '${q}_3$', '${q}_4$', '${q}_5$', '${q}_6$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');
xlabel('time (sec)','FontSize',13,...
       'FontWeight','bold', 'Interpreter', 'latex')
ylabel('${\theta} (rad)$','FontSize',13,...
       'FontWeight','bold', 'Interpreter', 'latex')
figure
plot(tvec, qdInterp)
grid on
title('Joint Velocities')
legend('$\dot{q}_1$', '$\dot{q}_2$', '$\dot{q}_3$', '$\dot{q}_4$', '$\dot{q}_5$', '$\dot{q}_6$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');
xlabel('time (sec)','FontSize',13,...
       'FontWeight','bold', 'Interpreter', 'latex')
ylabel('$\dot{\theta} (rad/s)$','FontSize',13,...
       'FontWeight','bold', 'Interpreter', 'latex')

figure
plot(tvec, qddInterp)
grid on
title('Joint Acceleration')
legend('$\ddot{q}_1$', '$\ddot{q}_2$', '$\ddot{q}_3$', '$\ddot{q}_4$', '$\ddot{q}_5$', '$\ddot{q}_6$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');
xlabel('time (sec)','FontSize',13,...
       'FontWeight','bold', 'Interpreter', 'latex')
ylabel('$\ddot{\theta} (rad/s^2)$','FontSize',13,...
       'FontWeight','bold', 'Interpreter', 'latex')

figure
plot(tvec, jointTorq)
grid on
title('joint Torque')
legend('${\tau}_1$', '${\tau}_2$', '${\tau}_3$', '${\tau}_4$', '${\tau}_5$', '${\tau}_6$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');
xlabel('time (sec)','FontSize',13,...
       'FontWeight','bold', 'Interpreter', 'latex')
ylabel('${\tau}$','FontSize',13,...
       'FontWeight','bold', 'Interpreter', 'latex')