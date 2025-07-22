clear
close all
clc
%% Load robot URDF
myRobot = importrobot('abbIrb120.urdf', 'DataFormat', 'row');
myRobot.Gravity = [0 0 -9.81];

%% Using rottraj and Cartesian trajectory
% Define waypoints
orientations = [0         0         0
                pi/2      pi/6      pi/1.5
                0         0         0];

wpts = [0.25   0.25    0.25
       -0.40   0.00    0.40
        0.30   0.50    0.30];

waypointVels = 0.5 * [0   0   0
                      0   1   0
                      0   0   0];

waypointAccels = 0.1 * [0   0   0
                        0   1   0
                        0   0   0];

waypointJerks = 0.0 * [0   0   0
                       0   1   0
                       0   0   0];

% Define trajectory parameters
numWpts = size(wpts, 2);
numSample = 51;
totalSamples = numSample * (numWpts - 1); % Total samples across all segments
tvecGlobal = linspace(0, numWpts - 1, totalSamples); % Global time vector
waypointTimes = linspace(0, numWpts - 1, numWpts);
tvec = linspace(0, 1, numSample); % Time vector
%% Initialize arrays for all segments
qInterp = zeros(6, totalSamples);
qdInterp = zeros(6, totalSamples);
qddInterp = zeros(6, totalSamples);
jointTorq = zeros(6, totalSamples);
a = zeros(6, totalSamples);
v = zeros(6, totalSamples);
singularityThreshold = 0.00192;

%% Trajectory following loop
sampleIdx = 1; % Index to keep track of position in global arrays
for w = 1:numWpts-1
    % Get the initial and final rotations and times for the segment
    R0 = eul2quat(orientations(:, w)');
    Rf = eul2quat(orientations(:, w + 1)');

    % [p,pd,pdd] = quinticpolytraj(wpts(:,w:w+1),[0 1],tvec,...
    %     'VelocityBoundaryCondition', waypointVels(:,w:w+1), ...
    %     'AccelerationBoundaryCondition', waypointAccels(:,w:w+1));

    % Generate trajectory for the segment
    % [p, pd, pdd] = minjerkpolytraj(wpts(:, w:w+1), [0 1], numSample, ...
    %     'VelocityBoundaryCondition', waypointVels(:, w:w+1), ...
    %     'AccelerationBoundaryCondition', waypointAccels(:, w:w+1), ...
    %     'JerkBoundaryCondition', waypointJerks(:, w:w+1));

    [p,pd,pdd] = minsnappolytraj(wpts(:,w:w+1),[0 1],numSample,...
        'VelocityBoundaryCondition', waypointVels(:,w:w+1), ...
        'AccelerationBoundaryCondition', waypointAccels(:,w:w+1), ...
        'JerkBoundaryCondition', waypointJerks(:, w:w+1));
    
    % Generate rotation trajectory
    [s, sd, sdd] = minsnappolytraj([0 1], [0 1], numSample);
    [R, omega, alpha] = rottraj(R0, Rf, [0 1], tvec, 'TimeScaling', [s; sd; sdd]);
    
    % Process each sample in the segment
    for i = 1:numSample
        tfInterp = trvec2tform(p(:, i)') * quat2tform(R(:, i)');
        q0 = ikine_abbIrb120(tfInterp, false, false);
        jacobian = geometricJacobian(myRobot, q0(1, :), 'tool0');
        v(:, sampleIdx) = [omega(:, i); pd(:, i)];
        a(:, sampleIdx) = [alpha(:, i); pdd(:, i)];
        qdInterp(:, sampleIdx) = jacobian \ v(:, sampleIdx);
        qddInterp(:, sampleIdx) = jacobian \ a(:, sampleIdx);
        jointTorq(:, sampleIdx) = inverseDynamics(myRobot, q0(1, :), qdInterp(:, sampleIdx)', qddInterp(:, sampleIdx)');
        qInterp(:, sampleIdx) = q0(1, :);
        
        % Visualization
        show(myRobot, qInterp(:, sampleIdx)', 'Frames', 'off', 'PreservePlot', false, 'FastUpdate', true);
        xlim([-0.7 0.7]), ylim([-0.7 0.7]), zlim([0 1])
        drawnow
        hold on
        if i == 1
            plot3(wpts(1, :), wpts(2, :), wpts(3, :), 'ro', 'LineWidth', 2);
            xlim([-0.7 0.7]), ylim([-0.7 0.7]), zlim([0 1])
            grid on
            hold on
        end
        pos = tform2trvec(tfInterp);
        plot3(pos(1), pos(2), pos(3), 'b.-', 'LineWidth', 4, 'MarkerSize', 3, 'MarkerFaceColor', 'r');
        grid on
        
        sampleIdx = sampleIdx + 1; % Increment global index
    end
end

%% Plot results
figure
plot(tvecGlobal, qInterp)
grid on
title('Joint Interpolated')
legend('${q}_1$', '${q}_2$', '${q}_3$', '${q}_4$', '${q}_5$', '${q}_6$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');

figure
plot(tvecGlobal, qdInterp)
grid on
title('Joint Velocities')
legend('$\dot{q}_1$', '$\dot{q}_2$', '$\dot{q}_3$', '$\dot{q}_4$', '$\dot{q}_5$', '$\dot{q}_6$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');

figure
plot(tvecGlobal, qddInterp)
grid on
title('Joint Accelerations')
legend('$\ddot{q}_1$', '$\ddot{q}_2$', '$\ddot{q}_3$', '$\ddot{q}_4$', '$\ddot{q}_5$', '$\ddot{q}_6$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');

figure
plot(tvecGlobal, v(4:6, :))
grid on
title('End-Effector Velocities')
legend('$\dot{x}$', '$\dot{y}$', '$\dot{z}$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');

figure
plot(tvecGlobal, v(1:3, :))
grid on
title('Orientation Velocities')
legend('$\dot{\phi}$', '$\dot{\theta}$', '$\dot{\psi}$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');

figure
plot(tvecGlobal, a(4:6, :))
grid on
title('End-Effector Accelerations')
legend('$\ddot{x}$', '$\ddot{y}$', '$\ddot{z}$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');

figure
plot(tvecGlobal, a(1:3, :))
grid on
title('Orientation Accelerations')
legend('$\ddot{\phi}$', '$\ddot{\theta}$', '$\ddot{\psi}$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');

figure
plot(tvecGlobal, jointTorq)
grid on
title('Joint Torques')
legend('${\tau}_1$', '${\tau}_2$', '${\tau}_3$', '${\tau}_4$', '${\tau}_5$', '${\tau}_6$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');