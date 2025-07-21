clear
close all
clc
%% Load robot URDF
Robot = importrobot('myRobot.urdf', 'DataFormat', 'row');
Robot.Gravity =  [0 0 -9.81];

runTrajMode = trajMode.minsnappolytraj;
runP2pMode = p2pMode.moveL;
wpts = [
    0.18    0.18
   -0.1851     0.1851
    0.1000    0.1000
    ];
orientations = [0         0
                pi/2      pi/2
                0         0];

waypointVels = 0.0 * [0   0
                      0   1
                      0   0];

waypointAccels = 0.0 * [0   0
                        0   1
                        0   0];

waypointJerks = 0.0 * [0   0
                       0   1
                       0   0];
tvec = linspace(0, 1, 101); % Time vector
[pos, qInterp, qdInterp,jointTorq] = run_p2p(wpts,orientations,runP2pMode,runTrajMode,waypointVels,waypointAccels,waypointJerks);
for i = 1:101
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
    plot3(pos(i,1), pos(i,2), pos(i,3), 'b.-', 'LineWidth', 4, 'MarkerSize', 3, 'MarkerFaceColor', 'r');
    grid on
end

% %% Plot results
% min(min_singular_value)
% figure
% plot(tvec, min_singular_value)
% grid on

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
% 
% figure
% plot(tvec, qddInterp)
% grid on
% title('Joint Acceleration')
% legend('$\ddot{q}_1$', '$\ddot{q}_2$', '$\ddot{q}_3$', '$\ddot{q}_4$', '$\ddot{q}_5$', '$\ddot{q}_6$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');
% xlabel('time (sec)','FontSize',13,...
%        'FontWeight','bold', 'Interpreter', 'latex')
% ylabel('$\ddot{\theta} (rad/s^2)$','FontSize',13,...
%        'FontWeight','bold', 'Interpreter', 'latex')
% 
% figure
% plot(tvec, jointTorq)
% grid on
% title('joint Torque')
% legend('${\tau}_1$', '${\tau}_2$', '${\tau}_3$', '${\tau}_4$', '${\tau}_5$', '${\tau}_6$', 'Location', 'best', 'Interpreter', 'latex', 'FontSize', 13, 'FontWeight', 'bold');
% xlabel('time (sec)','FontSize',13,...
%        'FontWeight','bold', 'Interpreter', 'latex')
% ylabel('${\tau}$','FontSize',13,...
%        'FontWeight','bold', 'Interpreter', 'latex')