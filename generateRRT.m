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
            disp("found node")
        end
        if can_extend  % can it extend
            % add new_node to start tree
            vertex_start = vertcat(vertex_start, new_node);
            edges_start = vertcat(edges_start, [index1, length(vertex_start)]);
            % plotting tree
            StartLines = animatedline('color', 'k', 'linewidth',1.5);
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
h = animatedline('color', 'r', 'LineWidth', 2);
for i = 1:size(PATH, 1)
    addpoints(h, PATH(i,1), PATH(i,2), PATH(i,3));
    drawnow limitrate
end
drawnow
for i = 1:size(PATH, 1)
    %temp = main_shaft
    matZ = axang2tform([0 1 0 PATH(i,4)]);
    matZ(1,4) = PATH(i,1);
    matZ(2,4) = PATH(i,2);
    matZ(3,4) = PATH(i,3);
    main_shaft.Pose = matZ;
    main_shaft_bearings.Pose = matZ;

    [~,patchObj] = show(main_shaft);
    patchObj.FaceColor = [0 0 1];
    [~,patchObj] = show(main_shaft_bearings);
    patchObj.FaceColor = [0 0 1];
    pause(0.05)
end
end

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