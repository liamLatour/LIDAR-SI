%% EXAMPLE: Differential drive navigation
% In this example, path is found in an occupancy grid using a probabilistic
% roadmap (robotics.PRM) and followed using Pure Pursuit (robotics.PurePursuit)
% 
% Copyright 2018 The MathWorks, Inc.

%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Instantiate LIDAR
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(0,pi*2,181);
lidar.maxRange = 4;

%% Simulation parameters
sampleTime = 0.05;             % Sample time [s]
tVec = 0:sampleTime:20;         % Time array

initPose = [10;10;0];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

%% Path planning
% Load map and inflate it by a safety distance
close all
load exampleMap
inflate(map,R);

% Create a Probabilistic Road Map (PRM)
planner = robotics.PRM(map);
planner.NumNodes = 75;
planner.ConnectionDistance = 5;

% Find a path from the start point to a specified goal point
startPoint = initPose(1:2)';
goalPoint  = [2, 1];
waypoints = findpath(planner,startPoint,goalPoint);

%% Pure Pursuit Controller
controller = robotics.PurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.35;
controller.DesiredLinearVelocity = 1.75;
controller.MaxAngularVelocity = 2.5;

%% Known Map
resolution = 20;
mapSize = 13.5;
knownMap = robotics.BinaryOccupancyGrid(mapSize,mapSize,resolution);
[x,y] = meshgrid(0:1/(resolution+1):mapSize);
setOccupancy(knownMap, [x(:) y(:)] ,1);

%% Walls Map
thisRes = 4;
wallsMap = robotics.BinaryOccupancyGrid(mapSize,mapSize,thisRes);
knownHoles = [];

%% Create visualizer 
load exampleMap % Reload original (uninflated) map for visualization
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

myFigure = figure('Name', 'Maps','NumberTitle','off');
figure(myFigure);
ax1 = subplot(2, 2, 1);
ax2 = subplot(2, 2, 2);
ax3 = subplot(2, 2, 4);
show(planner, "Parent", ax1)
grid(ax3, 'on')
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0, 1, 1]);
set(ax3,'XTick',0:1:14,'YTick',0:1:14);

%% Simulation loop
r = robotics.Rate(1/sampleTime);
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
    
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization and LIDAR
    ranges = lidar(pose(:,idx));
    hasBeenNan = false;    
    
    for i = 1 : length(ranges)
        range = ranges(i);
        angle = (i-1)*(2*pi)/(180) + pose(3,idx);
        
        if isnan(range)
           range = lidar.maxRange-0.1;
           if hasBeenNan == false
               hasBeenNan = (i-2)*(2*pi)/(180) + pose(3,idx);
           end
        else
            if hasBeenNan ~= false
                newAngle = (angle+hasBeenNan)/2;                
                newA = pol2car([lidar.maxRange newAngle], [pose(1,idx) pose(2,idx)], [0 mapSize], thisRes);
                
                
                if getOccupancy(knownMap, newA) ~= 0
                    knownHoles(end+1, :) = [pose(1,idx), pose(2,idx), newAngle];%newA; % VERY NOT OPTIMIZED
                end
                hasBeenNan = false;
            end
            xy = pol2car([range angle], [pose(1,idx) pose(2,idx)], [0 mapSize], thisRes);
            setOccupancy(wallsMap, xy, 1);
        end
        
        x = (cos(angle)*range+pose(1,idx)) * resolution;
        y = (sin(angle)*range+pose(2,idx)) * resolution;
        
        [posX, posY] = bresenham(pose(1,idx)* resolution, pose(2,idx)* resolution, x, y);
        a = min(max([posX, posY]/resolution, 0), mapSize);
        
        setOccupancy(knownMap, a, 0);
    end
    
    % Quick cleanUp
    for i = size(knownHoles, 1):-1:1        
        newA = pol2car([lidar.maxRange knownHoles(i,3)], [knownHoles(i,1) knownHoles(i,2)], [0 mapSize], thisRes);
        
        if getOccupancy(knownMap, newA) == 0
            knownHoles(i, :) = [];
        end
    end
    
    % Display everything
    viz(pose(:,idx), waypoints, ranges)
    figure(myFigure);
    show(knownMap, "Parent", ax2);
    show(wallsMap, "Parent", ax3);
    grid(ax3, 'on')
    set(ax3,'XTick',0:1:14,'YTick',0:1:14);
    hold on
    
    %disp(knownHoles);
    %disp("Size " + size(knownHoles, 1));
    distance = [];
    distance(1:size(knownHoles, 1), 1) = lidar.maxRange-0.2;
    %disp(distance);
    cartesian = pol2car( [distance knownHoles(:, 3)], [knownHoles(:, 1) knownHoles(:, 2)], [0 mapSize], thisRes);
    plot(cartesian(:, 1) , cartesian(:, 2) , '.');
    
    waitfor(r);
end