function [pose, qInterp, qdInterp,jointTorq] = run_p2p(wpts,orientations,runP2pMode,runTrajMode,waypointVels,waypointAccels,waypointJerks)
    % Load robot URDF
    robot = importrobot('myRobot.urdf', 'DataFormat', 'row');
    robot.Gravity =  [0 0 -9.81];
    % Define trajectory parameters
    numSample = int8(101);
    tvec = linspace(0, 1, numSample); % Time vector

    % Parse optional inputs
    narginchk(1,7);
    if nargin < 7
        waypointJerks = zeros(3,2);
    end
    if nargin < 6
        waypointAccels = zeros(3,2);
    end
    if nargin < 5
        waypointVels = zeros(3,2);
    end
    if nargin < 4
        runTrajMode = trajMode.trapveltraj;
    end
    if nargin < 3
        runP2pMode = p2pMode.moveL;
    end    
    %%
    % Trajectory following loop
    %% Initialize arrays
    pose = zeros(numSample,3);
    qInterp = zeros(6, numSample);
    qdInterp = zeros(6, numSample);
    qddInterp = zeros(6, numSample);
    jointTorq = zeros(6, numSample);
    min_singular_value = zeros(1, numSample);
    singularityThreshold = 0.005;
    
    % Get the initial and final rotations and times for the segment
    R0 = eul2quat(orientations(:,1)');
    Rf = eul2quat(orientations(:,2)');
    switch runTrajMode
        case trajMode.minjerkpolytraj
            [p,pd,pdd] = minjerkpolytraj(wpts,[0 1],numSample,...
                'VelocityBoundaryCondition', waypointVels, ...
                'AccelerationBoundaryCondition', waypointAccels, ...
                'JerkBoundaryCondition', waypointJerks);
        case trajMode.minsnappolytraj
            [p,pd,pdd] = minsnappolytraj(wpts,[0 1],numSample,...
                'VelocityBoundaryCondition', waypointVels, ...
                'AccelerationBoundaryCondition', waypointAccels, ...
                'JerkBoundaryCondition', waypointJerks);
        case trajMode.quinticpolytraj
            [p,pd,pdd] = quinticpolytraj(wpts,[0 1],tvec,...
                'VelocityBoundaryCondition', waypointVels, ...
                'AccelerationBoundaryCondition', waypointAccels);
        case trajMode.cubicpolytraj
            [p,pd,pdd] = cubicpolytraj(wpts,[0 1],tvec,...
                'VelocityBoundaryCondition', waypointVels);
        otherwise
            [p,pd,pdd] = trapveltraj(wpts, numSample);
    end
    switch runP2pMode
        case p2pMode.moveL
        case p2pMode.moveJ
        case p2pMode.moveJI
        otherwise
    end
    
    % Find the quaternions from trajectory generation
    [s, sd, sdd] = minjerkpolytraj([0 1], [0 1], numSample);
    
    % [s, sd, sdd] = quinticpolytraj([0 1], [0 1], tvec);
    [R, omega, alpha] = rottraj(R0, Rf, [0 1], tvec, 'TimeScaling', [s; sd; sdd]);
    
    for i = 1:numSample
        tfInterp = trvec2tform(p(:,i)') * quat2tform(R(:,i)');
        pose(i,:) = tform2trvec(tfInterp);
        q0 = ikine_myRobot(tfInterp, true, true, qInterp(6,i)');
        jacobian = geometricJacobian(robot, q0(1,:), 'tool0');
        qdInterp(:,i) = jacobian \ [omega(:,i); pd(:,i)];
        qddInterp(:,i) = jacobian \ [alpha(:,i); pdd(:,i)];
        jointTorq(:,i) = inverseDynamics(robot, q0(1,:), qdInterp(:,i)', qddInterp(:,i)');
        [~, S, ~] = svd(jacobian);
        min_singular_value(1,i) = min(diag(S));
        if min_singular_value(1,i) < singularityThreshold
            disp('The robot is at a singularity.');
        end
        qInterp(:, i) = q0(1,:);
    end
end