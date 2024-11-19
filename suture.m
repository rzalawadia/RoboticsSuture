% Set the home configuration and calculate the transformation at the home
% configuration
kr1 = importrobot('RightArm.urdf');    
kr1.DataFormat = 'column';
q_home1 = [0;0;0;0;0.7854;0]; % q5 ~ 45°
endEffector1 = 'needlink_5';
endEffector2 = 'needlink_5';
T_home1 = getTransform(kr1, q_home1, endEffector1);

kr2 = importrobot('LeftArm.urdf');
kr2.DataFormat = 'column';
q_home2 = [0;0;0;0;0.7854;0]; % q5~45°
T_home2 = getTransform(kr2, q_home2, endEffector2); 

% % Visualize the robot at home configuration
% show(kr,q_home);

% Create Inverse Kinematics Solver and Set Parameters
ik1 = inverseKinematics('RigidBodyTree', kr1);
ik2 = inverseKinematics('RigidBodyTree',kr2);

% Disable random restarts for inverse kinematic solver
ik1.SolverParameters.AllowRandomRestart = false;
ik2.SolverParameters.AllowRandomRestart = false;

% Specify weight priorities for pose tolerances
weights = [0.25, 0.25, 0.25, 1, 1, 1];
q_init1 = q_home1;
q_init2 = q_home2;

% Define Waypoints from the Desired Trajectory
% Suture path
starting_point1 = [2.6 0 0]; %[x y z]
starting_point2 = [2.6 0.2 0];

% Circular trajectory
% radius = 0.2;
% 
% % Define total time and time step to generate waypoints 
% dt = 0.20;
% t = (0:dt:10)';
% theta = t*(2*pi/t(end))-(pi/2);
% points = center + radius*[cos(theta) sin(theta) 0*ones(size(theta))];

% Zig-zag pattern
% Define Waypoints for Zig-Zag Pattern
num_segments = 1; % Adjust the number of zig-zag segments
num_points_per_segment = 50; % Adjust the number of points per segment
step_size = 0.2;

% Initialize points matrix
points1 = zeros(num_segments * num_points_per_segment, 3);
points2 = zeros(num_segments * num_points_per_segment, 3);

% Generate zig-zag pattern
for i = 1:num_segments
    
    % Diagonal movement (upward) 1st Arm
    points1((i - 1) * num_points_per_segment + 1:i * num_points_per_segment, :) = ...
        [linspace(starting_point1(1), starting_point1(1) + step_size, num_points_per_segment)', ...
         linspace(starting_point1(2), starting_point1(2) + step_size, num_points_per_segment)', ...
         starting_point1(3) + 0 * ones(num_points_per_segment, 1)];

    % Diagonal movement (downward) 2nd Arm
    points2(i * num_points_per_segment + 1:(i + 1) * num_points_per_segment, :) = ...
        [linspace(starting_point2(1) + step_size, starting_point2(1) + 2*step_size, num_points_per_segment)', ...
         linspace(starting_point2(2) - step_size, starting_point2(2), num_points_per_segment)', ...
         starting_point2(3) + 0 * ones(num_points_per_segment, 1)];

    % Diagonal movement (downward) 1st Arm
    points1(i * num_points_per_segment + 1:(i + 1) * num_points_per_segment, :) = ...
        [linspace(starting_point1(1) + step_size, starting_point1(1) + 2*step_size, num_points_per_segment)', ...
         linspace(starting_point1(2) + step_size, starting_point1(2), num_points_per_segment)', ...
         starting_point1(3) + 0 * ones(num_points_per_segment, 1)];

    % Diagonal movement (upward) 2nd Arm
    points2((i - 1) * num_points_per_segment + 1:i * num_points_per_segment, :) = ...
        [linspace(starting_point2(1), starting_point2(1) + step_size, num_points_per_segment)', ...
         linspace(starting_point2(2), starting_point2(2) - step_size, num_points_per_segment)', ...
         starting_point2(3) + 0 * ones(num_points_per_segment, 1)];
   
end

% Solve the Inverse Kinematics for Each Waypoint
numJoints = size(q_home1,1);   
numWaypoints = size(points1,1);
qs1 = zeros(numWaypoints,numJoints);
qs2 = zeros(numWaypoints,numJoints);

for i = 1:numWaypoints
    T_des1 = T_home1;
    T_des2 = T_home2;
    T_des1(1:3,4) = points1(i,:)';
    T_des2(1:3,4) = points2(i,:)';
    [q_sol1, q_info1] = ik1(endEffector1, T_des1, weights, q_init1);
    [q_sol2, q_info2] = ik2(endEffector2, T_des2, weights, q_init2);
   
    % Display status of ik result
    %disp(q_info.Status);

    % Store the configuration
    qs1(i,:) = q_sol1(1:numJoints); 
    qs2(i,:) = q_sol2(1:numJoints);

    % Start from prior solution
    q_init1 = q_sol1;
    q_init2 = q_sol2;
end

% Visualize the Animation of the Solution
figure(1); set(gcf,'Visible','on');
ax1 = show(kr1,qs1(1,:)');
ax2 = show(kr2,qs2(1,:)');
ax1.CameraPositionMode='auto';
ax2.CameraPositionMode='auto';
hold on;

% Initialize plot for waypoints
waypointsPlot1 = plot3(points1(:,1),points1(:,2),points1(:,3),'-r','LineWidth',2);
waypointsPlot2 = plot3(points2(:,1),points2(:,2),points2(:,3),'-r','LineWidth',2);
axis auto;
view([60,10]);
grid('minor');
hold on;

% Animate
framesPerSecond = 15;
r = robotics.Rate(framesPerSecond);

% % Set up VideoWriter
% videoFilename = 'suture_v3.mp4'; 
% videoWriter = VideoWriter(videoFilename, 'MPEG-4');
% videoWriter.FrameRate = framesPerSecond;
% open(videoWriter);

% Draw an initial patch
XL = get(gca, 'XLim');
YL = get(gca, 'YLim');
patch([XL(1), XL(2), XL(2), XL(1)], [YL(1), YL(1), YL(2), YL(2)], [0 0 0 0],'FaceColor','#ffdac6','EdgeColor', 'none');

for i = 1:numWaypoints
    show(kr1, qs1(i,:)','PreservePlot',false);
    show(kr2, qs2(i,:)','PreservePlot',false);
        
    % Update red lines for waypoints
    set(waypointsPlot1, 'XData', points1(1:i,1), 'YData', points1(1:i,2), 'ZData', points1(1:i,3));
    set(waypointsPlot2, 'XData', points2(1:i,1), 'YData', points2(1:i,2), 'ZData', points2(1:i,3));
  
    drawnow;
    
    % Keep drawing the patch if you move the XYZ plane during the motion
    XL = get(gca, 'XLim');
    YL = get(gca, 'YLim');
    patch([XL(1), XL(2), XL(2), XL(1)], [YL(1), YL(1), YL(2), YL(2)], [0 0 0 0],'FaceColor','#ffdac6','EdgeColor', 'none');

    % % Write the current frame to the video
    % currentFrame = getframe(gcf);
    % writeVideo(videoWriter, currentFrame);
    % pause(0.5);

    waitfor(r);
end

% close(videoWriter);