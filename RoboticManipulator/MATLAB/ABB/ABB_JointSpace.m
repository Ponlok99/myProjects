% clear
% close all
% clc
% %% Load robot URDF
% myRobot = importrobot('abbIrb120.urdf', 'DataFormat', 'row');
% myRobot.Gravity =  [0 0 -9.81];
% 
% %% Using rottraj and Cartesian trajectory
% % Define waypoints
% wptsJoints = [0 0 0 0 0 0
%               -pi/3 pi/5 pi/4 pi/3 pi/3 -pi/3;
%               pi/3 -pi/5 pi/4 -pi/3 pi/4 0;
%               0 0 0 0 0 0]';
% 
% wpts = zeros(3, size(wptsJoints,2));
% 
% waypointVels = zeros(size(wptsJoints));
% 
% waypointAccels = zeros(size(wptsJoints));
% 
% waypointJerks = zeros(size(wptsJoints));
% % Define trajectory parameters
% numWpts = size(wptsJoints,2);
% numSample = 51;
% tvec = linspace(0, 1, numSample); % Time vector
% waypointTimes = linspace(0, numWpts-1, numWpts);
% 
% %% Initialize arrays
% jointTorq = zeros(6, numSample);
% a = zeros(6, numSample);
% v = zeros(6, numSample);
% % d = zeros(1, numSample);
% singularityThreshold = 0.00192;
% 
% for w = 1:numWpts-1
%     [q,qd,qdd] = quinticpolytraj(wptsJoints(:,w:w+1),[0 1],tvec,...
%         'VelocityBoundaryCondition', waypointVels(:,w:w+1), ...
%         'AccelerationBoundaryCondition', waypointAccels(:,w:w+1));
%     % [q,qd,qdd] = minjerkpolytraj(wptsJoints(:,w:w+1),[0 1],numSample,...
%     %     'VelocityBoundaryCondition', waypointVels(:,w:w+1), ...
%     %     'AccelerationBoundaryCondition', waypointAccels(:,w:w+1), ...
%     %     'JerkBoundaryCondition', waypointJerks(:,w:w+1));    
%     % [p,pd,pdd] = minsnappolytraj(wpts(:,w:w+1),[0 1],numSample,...
%     %     'VelocityBoundaryCondition', waypointVels(:,w:w+1), ...
%     %     'AccelerationBoundaryCondition', waypointAccels(:,w:w+1));
%     T = getTransform(myRobot, wptsJoints(:,w)', 'tool0');
%     wpts(:,w) = tform2trvec(T)';
%     for i = 1:numSample
%         tfInterp = getTransform(myRobot, q(:,i)', 'tool0');
%         jacobian = geometricJacobian(myRobot, q(:,i)', 'tool0');
%         v(:,i) = jacobian *  qd(:,i);
%         a(:,i) = jacobian * qdd(:,i);
%         jointTorq(:,i) = inverseDynamics(myRobot, q(:,i)', qd(:,i)', qdd(:,i)');
%         % Visualization
%         show(myRobot, q(:, i)', 'Frames', 'off', 'PreservePlot', false,'FastUpdate',true);
%         xlim([-0.7 0.7]), ylim([-0.7 0.7]), zlim([0 1])
%         drawnow
%         hold on
%         if i == 1
%             plot3(wpts(1,:),wpts(2,:),wpts(3,:),'ro','LineWidth',2);
%             xlim([-0.7 0.7]), ylim([-0.7 0.7]), zlim([0 1])
%             grid on
%             hold on
%         end
%         pos = tform2trvec(tfInterp);
%         plot3(pos(1), pos(2), pos(3), 'b.-', 'LineWidth', 4, 'MarkerSize', 3, 'MarkerFaceColor', 'r');
%         grid on
%     end
% end
% 
% % Plot results
% figure
% plot(tvec, q)
% grid on
% title('Joint Interpolated')
% legend('${q}_1$', '${q}_2$', '${q}_3$', '${q}_4$', '${q}_5$', '${q}_6$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');
% 
% figure
% plot(tvec, qd)
% grid on
% title('Joint Velocities')
% legend('$\dot{q}_1$', '$\dot{q}_2$', '$\dot{q}_3$', '$\dot{q}_4$', '$\dot{q}_5$', '$\dot{q}_6$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');
% 
% figure
% plot(tvec, qdd)
% grid on
% title('Joint Acc')
% legend('$\ddot{q}_1$', '$\ddot{q}_2$', '$\ddot{q}_3$', '$\ddot{q}_4$', '$\ddot{q}_5$', '$\ddot{q}_6$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');
% 
% figure
% plot(tvec, v(4:6, :))
% grid on
% title('ee Velocities')
% legend('$\dot{x}$', '$\dot{y}$', '$\dot{z}$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');
% 
% figure
% plot(tvec, v(1:3, :))
% grid on
% title('orien Velocities')
% legend('$\dot{\phi}$', '$\dot{\theta}$', '$\dot{\psi}$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');
% 
% figure
% plot(tvec, a(4:6, :))
% grid on
% title('TCP Acc')
% legend('$\ddot{x}$', '$\ddot{y}$', '$\ddot{z}$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');
% 
% figure
% plot(tvec, a(1:3, :))
% grid on
% title('Orient Acc')
% legend('$\ddot{\phi}$', '$\ddot{\theta}$', '$\ddot{\psi}$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');
% 
% figure
% plot(tvec, jointTorq)
% grid on
% title('joint Torque')
% legend('${\tau}_1$', '${\tau}_2$', '${\tau}_3$', '${\tau}_4$', '${\tau}_5$', '${\tau}_6$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');


clear
close all
clc
%% Load robot URDF
myRobot = importrobot('abbIrb120.urdf', 'DataFormat', 'row');
myRobot.Gravity = [0 0 -9.81];

%% Using rottraj and Cartesian trajectory
% Define waypoints
wptsJoints = [0 0 0 0 0 0
              -pi/3 pi/5 pi/4 pi/3 pi/3 -pi/3
              pi/3 -pi/5 pi/4 -pi/3 pi/4 0
              0 0 0 0 0 0]';

wpts = zeros(3, size(wptsJoints,2));

waypointVels = zeros(size(wptsJoints));

waypointAccels = zeros(size(wptsJoints));

waypointJerks = zeros(size(wptsJoints));
% Define trajectory parameters
numWpts = size(wptsJoints,2);
numSample = 51;
tvec = linspace(0, 1, numSample); % Time vector for each segment
waypointTimes = linspace(0, numWpts-1, numWpts);

%% Initialize arrays
jointTorq = zeros(6, numSample * (numWpts-1));
a = zeros(6, numSample * (numWpts-1));
v = zeros(6, numSample * (numWpts-1));
q_all = zeros(6, numSample * (numWpts-1));
qd_all = zeros(6, numSample * (numWpts-1));
qdd_all = zeros(6, numSample * (numWpts-1));
tvec_all = zeros(1, numSample * (numWpts-1));
singularityThreshold = 0.00192;

% Counter for storing data across all segments
sampleIdx = 1;

for w = 1:numWpts-1
    % Generate trajectory for the current segment
    % [q, qd, qdd] = quinticpolytraj(wptsJoints(:,w:w+1), [0 1], tvec, ...
    %     'VelocityBoundaryCondition', waypointVels(:,w:w+1), ...
    %     'AccelerationBoundaryCondition', waypointAccels(:,w:w+1));
    % [q,qd,qdd] = minjerkpolytraj(wptsJoints(:,w:w+1),[0 1],numSample,...
    %     'VelocityBoundaryCondition', waypointVels(:,w:w+1), ...
    %     'AccelerationBoundaryCondition', waypointAccels(:,w:w+1), ...
    %     'JerkBoundaryCondition', waypointJerks(:,w:w+1));
    [q,qd,qdd] = minsnappolytraj(wptsJoints(:,w:w+1),[0 1],numSample,...
        'VelocityBoundaryCondition', waypointVels(:,w:w+1), ...
        'AccelerationBoundaryCondition', waypointAccels(:,w:w+1));
    % Adjust time vector for the current segment
    tvec_segment = tvec + (w-1); % Shift time to be cumulative
    
    % Store interpolated data
    q_all(:, sampleIdx:sampleIdx+numSample-1) = q;
    qd_all(:, sampleIdx:sampleIdx+numSample-1) = qd;
    qdd_all(:, sampleIdx:sampleIdx+numSample-1) = qdd;
    tvec_all(sampleIdx:sampleIdx+numSample-1) = tvec_segment;
    
    % Update waypoint Cartesian positions
    T = getTransform(myRobot, wptsJoints(:,w)', 'tool0');
    wpts(:,w) = tform2trvec(T)';
    
    for i = 1:numSample
        tfInterp = getTransform(myRobot, q(:,i)', 'tool0');
        jacobian = geometricJacobian(myRobot, q(:,i)', 'tool0');
        v(:,sampleIdx+i-1) = jacobian * qd(:,i);
        a(:,sampleIdx+i-1) = jacobian * qdd(:,i);
        jointTorq(:,sampleIdx+i-1) = inverseDynamics(myRobot, q(:,i)', qd(:,i)', qdd(:,i)');
        % Visualization
        show(myRobot, q(:,i)', 'Frames', 'off', 'PreservePlot', false, 'FastUpdate', true);
        xlim([-0.7 0.7]), ylim([-0.7 0.7]), zlim([0 1])
        drawnow
        hold on
        if i == 1
            plot3(wpts(1,:), wpts(2,:), wpts(3,:), 'ro', 'LineWidth', 2);
            xlim([-0.7 0.7]), ylim([-0.7 0.7]), zlim([0 1])
            grid on
            hold on
        end
        pos = tform2trvec(tfInterp);
        plot3(pos(1), pos(2), pos(3), 'b.-', 'LineWidth', 4, 'MarkerSize', 3, 'MarkerFaceColor', 'r');
        grid on
    end
    % Update sample index for the next segment
    sampleIdx = sampleIdx + numSample;
end

% Plot results for joint positions
figure
plot(tvec_all, q_all)
grid on
title('Joint Interpolated')
legend('${q}_1$', '${q}_2$', '${q}_3$', '${q}_4$', '${q}_5$', '${q}_6$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');

% Plot results for joint velocities
figure
plot(tvec_all, qd_all)
grid on
title('Joint Velocities')
legend('$\dot{q}_1$', '$\dot{q}_2$', '$\dot{q}_3$', '$\dot{q}_4$', '$\dot{q}_5$', '$\dot{q}_6$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');

% Plot results for joint accelerations
figure
plot(tvec_all, qdd_all)
grid on
title('Joint Accelerations')
legend('$\ddot{q}_1$', '$\ddot{q}_2$', '$\ddot{q}_3$', '$\ddot{q}_4$', '$\ddot{q}_5$', '$\ddot{q}_6$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');

% Plot results for end-effector linear velocities
figure
plot(tvec_all, v(4:6, :))
grid on
title('End-Effector Velocities')
legend('$\dot{x}$', '$\dot{y}$', '$\dot{z}$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');

% Plot results for end-effector orientation velocities
figure
plot(tvec_all, v(1:3, :))
grid on
title('Orientation Velocities')
legend('$\dot{\phi}$', '$\dot{\theta}$', '$\dot{\psi}$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');

% Plot results for end-effector linear accelerations
figure
plot(tvec_all, a(4:6, :))
grid on
title('TCP Accelerations')
legend('$\ddot{x}$', '$\ddot{y}$', '$\ddot{z}$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');

% Plot results for end-effector orientation accelerations
figure
plot(tvec_all, a(1:3, :))
grid on
title('Orientation Accelerations')
legend('$\ddot{\phi}$', '$\ddot{\theta}$', '$\ddot{\psi}$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');

% Plot results for joint torques
figure
plot(tvec_all, jointTorq)
grid on
title('Joint Torque')
legend('${\tau}_1$', '${\tau}_2$', '${\tau}_3$', '${\tau}_4$', '${\tau}_5$', '${\tau}_6$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');