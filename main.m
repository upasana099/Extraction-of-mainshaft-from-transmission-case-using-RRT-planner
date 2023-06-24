%% RBE 550 : Transmission Assignment
% Upasana Mahanti

clc;
clear all;
close all;

% First, we need to run the collision_checking script which contains the
% definitions of the obstacles
collision_checking 

% Define boundary for plotting and robot information
boundary = [10, 15, 8, 12, 10, 15, 0, pi/4]; % x_min, x_max, y_min, y_max, z_min, z_max, theta_min, theta_max
branch_length = 0.15; % Length of the branches of the tree

% Define the starting and goal points
start = [11.2, 10, 11, pi/2]; % starting point of center of main shaft
goal = [5, 10, 14, pi/2]; % goal point

% Plot the starting and goal points
hold on
view(0,0)
plot3(start(1), start(2), start(3), 'black*')
plot3(goal(1), goal(2), goal(3), 'red*')

% RRT planner to calculate collision free path


PATH = generateRRT(start, goal, box_obstacles, cylinder_obstacles, boundary, branch_length, main_shaft, main_shaft_bearings);

function PATH = generateRRT(start, goal, box_obstacles, cylinder_obstacles, boundary, branch_length,main_shaft, main_shaft_bearings)
% This function generates an RRT planner to calculate a collision-free path
% from the start point to the goal point.

% First, we check if the start and goal points are in obstacle-free space
if ~(collision_detection(start,box_obstacles, cylinder_obstacles) || collision_detection(goal, box_obstacles, cylinder_obstacles))
    N = 10000; % maximum number of samples to be generated
    vertex_start = start; % each tree vertex is a point in configuration space (q)
    edges_start = []; % edges connecting the nodes
    for i=1:N
        % search through configuration space
        if mod( i , 5) == 0
            % sample goal every 10 points
            rand_q = goal;
        else
            % sample random point
            x = rand(1,1)'.*(boundary(:,2)- boundary(:,1)) + boundary(:,1);
            y = rand(1,1)'.*(boundary(:,4)- boundary(:,3)) + boundary(:,3);
            z = rand(1,1)'.*(boundary(:,6)- boundary(:,5)) + boundary(:,5);
            theta = rand(1,1)'.*(boundary(:,8)- boundary(:,7)) + boundary(:,7);
            if rand <0.5
                theta = theta;
            else
                theta = -1*theta;
            end
            rand_q = [x y z theta];
        end
        % find nearest node in tree
        index1 = find_nearest_node(rand_q, vertex_start);
        q = vertex_start(index1, :);

        % Check if the node can be extended to the random node
        [can_extend, new_node] = can_it_extend(rand_q, q, box_obstacles, cylinder_obstacles, branch_length);
        if can_extend==1
            disp("node found!")
        end
        if can_extend  % can it extend
            % add new_node to start tree
            vertex_start = vertcat(vertex_start, new_node);
            edges_start = vertcat(edges_start, [index1, length(vertex_start)]);
            % plotting tree
            StartLines = animatedline('color', 'k', 'linewidth',1);
            xlabel('X')
            ylabel('Y')
            zlabel('Z')
            addpoints(StartLines, q(1), q(2), q(3));
            addpoints(StartLines, new_node(1), new_node(2), new_node(3));
            drawnow limitrate
            if round(new_node(1)) == goal(1) && round(new_node(2)) == goal(2)  &&  round(new_node(3)) == goal(3)  % if reached goal
                disp("Reached")
                break
            end
        end
    end
else
    PATH = [];
    disp('start or goal should not be in obstacle')
    return
end
% pick the best path in V and E
PATH = select_best_path(edges_start, vertex_start, goal);
display(PATH)
display(PATH(1,:))
% Plotting final path
%cla;
h = animatedline('color', 'r', 'LineWidth', 2);
for i = 1:size(PATH, 1)
    addpoints(h, PATH(i,1), PATH(i,2), PATH(i,3));
    drawnow limitrate
end
drawnow

for i = 1:size(PATH, 1)

    % Delete the previous main_shaft and main_shaft_bearings objects
    if i > 1
        delete(prev_main_shaft);
        %delete(prev_main_shaft_bearings);
    end

    % Set the new main_shaft and main_shaft_bearings poses
    matZ = axang2tform([0 1 0 PATH(i,4)]);
    matZ(1,4) = PATH(i,1);
    matZ(2,4) = PATH(i,2);
    matZ(3,4) = PATH(i,3);
    main_shaft.Pose = matZ;
    main_shaft_bearings.Pose = matZ;

    % Show the new main_shaft and main_shaft_bearings objects and store their handles
    [prev_main_shaft,patchObj] = show(main_shaft);
    patchObj.FaceColor = [0 0 1];
    [prev_main_shaft_bearings,patchObj] = show(main_shaft_bearings);
    patchObj.FaceColor = [0 0 1];

    pause(0.5)
end
end

% for i = 1:size(PATH, 1)
%     %temp = main_shaft
%     matZ = axang2tform([0 1 0 PATH(i,4)]);
%     matZ(1,4) = PATH(i,1);
%     matZ(2,4) = PATH(i,2);
%     matZ(3,4) = PATH(i,3);
%     main_shaft.Pose = matZ;
%     main_shaft_bearings.Pose = matZ;
% 
%     [~,patchObj] = show(main_shaft);
%     patchObj.FaceColor = [0 0 1];
%     [~,patchObj] = show(main_shaft_bearings);
%     patchObj.FaceColor = [0 0 1];
%     pause(0.05)
% end
% end

function closest_node_idx = find_nearest_node(q_rand, V)
% This function finds the index of the node in V that is closest to q_rand.
% Repeat q_rand for each row in V to create a matrix of the same size as V.
q = repmat(q_rand, size(V, 1), 1);
% Calculate the Euclidean distances between each row of V and q.
d = sqrt(sum((V - q).^2, 2)); 
% Find the index of the row in V with the smallest distance to q_rand.
[~, closest_node_idx] = min(d);
end

function [bool, new_node] = can_it_extend(rand_q, q, box_obstacles, cylinder_obstacles, branch_length)
% find new point fixed length away in direction of random point
dir_vector = (rand_q - q) / norm(rand_q - q);
potential_q = q + dir_vector * branch_length;
bool = true;
flag = 0;
new_node = potential_q;
% interpolate path and check for collision in each config waypoint
for x = linspace(potential_q(1), q(1), 5)
    for y = linspace(potential_q(2), q(2), 5)
        for z = linspace(potential_q(3), q(3), 5)
            if collision_detection([x y z potential_q(4)],box_obstacles, cylinder_obstacles)
                flag = 1;
                bool = false;
                break
            end
        if (flag == 1)
            break
        end
    if (flag == 1)
        break
    end
if (flag == 1)
        break
end
            if all(round([x, y, z, potential_q(4)]) == rand_q) % if branch passes through point
                new_node = rand_q;
                flag = 1;
                break
            end
        end
    end
end
end


function best_path = select_best_path(edges_start, vertex_start, goal)
start_path = [];
all(round(vertex_start(end, :), 1) == goal) % if tree reached goal
row = size(vertex_start, 1);
for z = 1:size(edges_start, 1)
    start_path = vertcat(start_path, vertex_start(row, :));
    next_row = edges_start(row-1, 1);
    if next_row == 1
         break
    else
            row = next_row;
    end
end
    % Combine and reorient
    start_path = vertcat(start_path, vertex_start(1, :));
    best_path = flip(start_path);
end

function collision = collision_detection(pose, box_obstacles,cylinder_obstacles)
    temp_shaft = collisionCylinder(0.2,3.4);
    temp_shaft_bearnings = collisionCylinder(0.5,2);
    matZ = axang2tform([0 1 0 pose(4)]);
    matZ(1,4) = pose(1);
    matZ(2,4) = pose(2);
    matZ(3,4) = pose(3);
    temp_shaft.Pose = matZ;
    temp_shaft_bearnings.Pose = matZ;
    collision = false;
    for i=1:length(box_obstacles)
        if(checkCollision(temp_shaft,box_obstacles(i)) ==1 || checkCollision(temp_shaft_bearnings,box_obstacles(i)) ==1)
            collision = true;
            return
        end
    end
    for i=1:length(cylinder_obstacles)
        if(checkCollision(temp_shaft,cylinder_obstacles(i)) ==1 || checkCollision(temp_shaft_bearnings,cylinder_obstacles(i)) ==1)
            collision = true;
            return
        end
    end
end