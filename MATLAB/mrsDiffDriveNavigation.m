%% EXAMPLE: Differential drive navigation
% In this example, path is found in an occupancy grid using a probabilistic
% roadmap (robotics.PRM) and followed using Pure Pursuit (robotics.PurePursuit)
% 
% Copyright 2018 The MathWorks, Inc.


%% Update made by Liam
%
% - Only relies on the known map
% - It goes to the place it has seen undiscovered places 
% - Goes to its direction
% - Repeats

close all
rng(0);

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
sampleTime = 0.1;             % Sample time [s]
tVec = 0:sampleTime:500;         % Time array

initPose = [10; 10; pi];      % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

%% Pure Pursuit Controller
controller = robotics.PurePursuit;
%controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.35;
controller.DesiredLinearVelocity = 1;
controller.MaxAngularVelocity = 2*pi;

blindDistance = 2;

%% Known Map
resolution = 20;
mapSize = 13.5;
knownMap = robotics.BinaryOccupancyGrid(mapSize,mapSize,resolution);
[x,y] = meshgrid(0:1/(resolution+1):mapSize);
setOccupancy(knownMap, [x(:) y(:)] ,1);

%% Walls Map
thisRes = 4;
wallsMap = robotics.BinaryOccupancyGrid(mapSize,mapSize,thisRes);
knownHoles = zeros(1000, 3);
currentHole = 1;

%% Create visualizer 
load exampleMap % Reload original (uninflated) map for visualization
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Initialize figures
myFigure = figure('Name','Maps', 'NumberTitle','off');
figure(myFigure);
ax1 = subplot(2, 2, 1);
ax2 = subplot(2, 2, 2);
ax3 = subplot(2, 2, 4);
ax4 = subplot(2, 2, 3);
%grid(ax3, 'on')
%set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0, 1, 1]);

%subplot(2, 2, 3)
%barGraph = bar(ax4, zeros(182));

%% Variables for the discovery code
diseredAngle = pi+0.7;
safeAngle = 10;
currentState = "start";
obstacleAvoidance = false;
waypoints = [];

%% Simulation loop
r = robotics.Rate(1/sampleTime);
for idx = 2:numel(tVec)     
    %% Part about moving
    if currentState == "obstacle"
        diffe = mod(diseredAngle-pose(3,idx-1), 2*pi);
        
        if diffe > pi
           diffe = diffe - 2*pi;
        end
        
        pose(3,idx) = mod(pose(3,idx-1) + min(max(diffe, -controller.MaxAngularVelocity/13), controller.MaxAngularVelocity/13), 2*pi);
        pose(1:2,idx) = pose(1:2,idx-1) + (pol2car([controller.DesiredLinearVelocity pose(3,idx)])*sampleTime)';        
    elseif currentState == "pursuit"
        % Run the Pure Pursuit controller and convert output to wheel speeds
        [vRef,wRef] = controller(pose(:,idx-1));
        [wL,wR] = inverseKinematics(dd,vRef,wRef);

        % Compute the velocities
        [v,w] = forwardKinematics(dd,wL,wR);
        velB = [v;0;w]; % Body velocities [vx;vy;w]
        vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world

        % Perform forward discrete integration step
        pose(:,idx) = pose(:,idx-1) + vel*sampleTime;
    elseif currentState == "blind"
        % Go to the dark direction
        diffe = mod(waypoint(3)-pose(3,idx-1), 2*pi);
        
        if diffe > pi
           diffe = diffe - 2*pi;
        end
        
        pose(3,idx) = mod(pose(3,idx-1) + min(max(diffe, -controller.MaxAngularVelocity/13), controller.MaxAngularVelocity/13), 2*pi);        
        pose(1:2,idx) = pose(1:2,idx-1) + (pol2car([controller.DesiredLinearVelocity pose(3,idx-1)])*sampleTime)';
    elseif currentState == "start"
        pose(:,idx) = pose(:,idx-1);
    end
    
    
    % Update the maps and visualization and LIDAR
    ranges = lidar(pose(:,idx));
    ranges(isnan(ranges))=4.1;
    hasBeenNan = false;
    
    for i = 1 : length(ranges)
        range = ranges(i);
        angle = (i-1)*(2*pi)/(180) + pose(3,idx);
        
        if range > 4
           range = lidar.maxRange-0.2;
           if hasBeenNan == false
               hasBeenNan = (i-2)*(2*pi)/(180) + pose(3,idx);
           end
        else
            if hasBeenNan ~= false
                newAngle = (angle+hasBeenNan)/2;                
                newA = pol2car([lidar.maxRange newAngle], pose(1:2,idx)', [0 mapSize], thisRes);
                
                if getOccupancy(knownMap, newA) ~= 0
                    knownHoles(currentHole, :) = [pose(1,idx), pose(2,idx), newAngle];
                    currentHole = currentHole+1;
                end
                hasBeenNan = false;
            end
            xy = pol2car([range angle], pose(1:2,idx)', [0 mapSize], thisRes);
            setOccupancy(wallsMap, xy, 1);
        end
        
        x = (cos(angle)*range+pose(1,idx)) * resolution;
        y = (sin(angle)*range+pose(2,idx)) * resolution;
        
        [posX, posY] = bresenham(pose(1,idx)* resolution, pose(2,idx)* resolution, x, y);
        a = min(max([posX, posY]/resolution, 0), mapSize);
        
        setOccupancy(knownMap, a, 0);
    end
    
    % Display everything
    figure(myFigure);
    
    if size(knownHoles, 1) > 0
        distance = [];
        distance(1:size(knownHoles, 1), 1) = lidar.maxRange-0.2;
        cartesian = pol2car( [distance knownHoles(:, 3)], knownHoles(:, 1:2), [0 mapSize], thisRes);
        %plot(cartesian(:, 1) , cartesian(:, 2) , '.');
    end
    
    if obstacleAvoidance == true || currentState == "obstacle"
        %show(knownMap, "Parent", ax2);
        %show(wallsMap, "Parent", ax3);
        
        nbScans = floor((length(lidar.scanAngles)+1)/2)*2;
        
        fours = ones(length(ranges), 1) * 4.1;
        temp = (fours-ranges)*50;
        splitMerge = [temp(nbScans/2:length(temp)); temp(1:nbScans/2)];
        
        % Check it can continue forward
        canContinue = true;
        for i = -safeAngle/2 : safeAngle/2
            if splitMerge(nbScans/2 + i) > 150
                canContinue = false;
                disp("NOPE");
                break;
            end
        end
        
        if canContinue == false
            % find closest available spot
            newAngleIndex = -1;
            %{ 
            % Old method, ineficient (constantly O(n))
            lastCheck = false;
            nbGood = 0;
            for i = 1 : length(splitMerge)
                if splitMerge(i) < 150
                    if lastCheck == true
                        
                        if abs(91-(i-safeAngle/2)) < abs(91-newAngleIndex)
                            newAngleIndex = i-safeAngle/2;
                        end
                    else
                        if nbGood < safeAngle
                            nbGood = nbGood + 1;
                        else
                            lastCheck = true;
                            if abs(91-(i-safeAngle/2)) < abs(91-newAngleIndex)
                                newAngleIndex = i-safeAngle/2;
                            end
                        end
                    end
                else
                    nbGood = 0;
                    lastCheck = false;
                end
            end
            %}
            
            % New method (worst O(n), best O(2*safeAngle))
            nbGood = 0;
            for i = (length(splitMerge)+safeAngle)/2 : length(splitMerge)
                if splitMerge(i) < 150
                    nbGood = nbGood + 1;
                    if nbGood == safeAngle
                        newAngleIndex = i-safeAngle/2;
                        break
                    end
                else
                    nbGood = 0;
                end
            end
            if newAngleIndex == -1
                newAngleIndex = nbScans;
            end
            nbGood = 0;
            for i = (length(splitMerge)-safeAngle)/2 : -1 : nbScans-newAngleIndex-safeAngle/2
                if splitMerge(i) < 150
                    nbGood = nbGood + 1;
                    if nbGood == safeAngle
                        newAngleIndex = i+safeAngle/2;
                        break
                    end
                else
                    nbGood = 0;
                end
            end
            
            splitMerge(newAngleIndex) = 250;
            barGraph = bar(ax4, splitMerge, 1);
            foundAngle = deg2rad(2*(newAngleIndex-nbScans/2));
            
            diseredAngle = foundAngle + pose(3,idx);
        end
    end
        
    if currentState == "start" && size(knownHoles, 1) > 0
        show(knownMap, "Parent", ax2);
        show(wallsMap, "Parent", ax3);
        grid(ax3, 'on')
        set(ax3,'XTick',0:1:14,'YTick',0:1:14);
        
        % Inflate
        oldMap = copy(knownMap);
        inflate(knownMap,0.3);
            
        % Chose a waypoint
        newCurrentHole = currentHole;
        waypoint = -1;
        
        % Quick cleanUp && choose waypoint
        for i = currentHole:-1:1        
            newA = pol2car([lidar.maxRange knownHoles(i,3)], [knownHoles(i,1) knownHoles(i,2)], [0 mapSize], thisRes);
            if getOccupancy(knownMap, newA) == 0
                knownHoles(i, :) = [];
                newCurrentHole = newCurrentHole-1;
            elseif getOccupancy(knownMap, knownHoles(i, 1:2)) == 0
                if waypoint == -1
                    waypoint = knownHoles(i, :);
                elseif pdist([knownHoles(i, 1:2); pose(1:2,idx)']) < pdist([waypoint(1:2); pose(1:2,idx)'])
                    waypoint = knownHoles(i, :);
                end
            end
        end
        currentHole = newCurrentHole;
        
        if waypoint == -1
            knownMap = copy(oldMap); % Deflate
            return;
        end
        
         % Check it really needs this
        if pdist([waypoint(1:2); pose(1:2,idx)']) < 0.5
            %pose(3,idx) = waypoint(3);
            currentState = "blind";
        else        
            % Create a Probabilistic Road Map (PRM)
            planner = robotics.PRM(knownMap);
            planner.NumNodes = 50;
            %planner.ConnectionDistance = 7;

            % Find a path from the start point to a specified goal point
            startPoint = pose(1:2,idx)';
            goalPoint = waypoint(1:2);
            waypoints = findpath(planner,startPoint,goalPoint);

            % Pure Pursuit Controller
            controller.Waypoints = waypoints;

            show(planner, "Parent", ax1)
            currentState = "pursuit";
        end
        % Deflate
        knownMap = copy(oldMap);
    elseif currentState == "pursuit"
        if pdist([waypoint(1:2); pose(1:2,idx)']) < 0.5
            % Quick cleanUp && check if mine is erased
            newCurrentHole = currentHole;
            erased = false;
            for i = currentHole:-1:1        
                newA = pol2car([lidar.maxRange knownHoles(i,3)], [knownHoles(i,1) knownHoles(i,2)], [0 mapSize], thisRes);
                if getOccupancy(knownMap, newA) == 0
                    if knownHoles(i, :) == waypoint
                       erased = true; 
                    end
                    knownHoles(i, :) = [];
                    newCurrentHole = newCurrentHole-1;
                end
            end
            currentHole = newCurrentHole;
            
            if erased
                currentState = "start";
            else
                %pose(3,idx) = waypoint(3);
                currentState = "blind";
            end
        end
    elseif currentState == "blind"
        if pdist([waypoint(1:2); pose(1:2,idx)']) > blindDistance
            currentState = "start";
        end
    end
    
    viz(pose(:,idx), waypoints, ranges);
    
    waitfor(r);
end