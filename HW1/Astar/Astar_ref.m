map = read_map_for_dynamics('maze1.pgm');
h_type = 'e';
% initialization
[start, num_nodes] = get_start_dynamic(map);
goal = get_goal_dynamic(map);
priority_list = inf * ones(num_nodes, 1); % list containing priorities
open_list = pq_init(1e+4); % open list
closed_list = []; % closed list
back_pointer = nan * ones(num_nodes, 1); % back-pointer attribute
% starting position
priority_list(start) = h(start, map, h_type);
open_list = pq_set(open_list, start, priority_list(start));
back_pointer(start) = -1;
path_cell = {};
path_cell{1} = start;

while true
    % pick best node and remove it from open_list
    [open_list, n_best] = pq_pop(open_list);
    % add best node to closed_list
    closed_list = [closed_list n_best];

    % check n_best is goal or not
    if n_best == goal
        break
    end

    % expand all nodes that are neibhbors of n_best
    [neighbors, ~] = get_neighbors_dynamic(map, n_best);
    neighbors = neighbors((neighbors == n_best) ~= 1);
    for index_neighbor=1:length(neighbors)
        if ismember(neighbors(index_neighbor), closed_list) == 1
            continue
        else
            neighbor = neighbors(index_neighbor);
            % calculate each cost
            cost_g = g(n_best, priority_list, map, h_type);
            cost_n = 1; % cost(n_best, neighbor, map);
            disp(cost(n_best, neighbor, map))
            cost_h = h(neighbor, map, h_type);
            priority = cost_g + cost_n + cost_h;
            test = pq_test(open_list, neighbor);
            if test == 1
                if priority < pq_priority(open_list, neighbor)
                    priority_list(neighbor) = priority;
                    open_list = pq_set(open_list, neighbor, priority);
                    back_pointer(neighbor) = n_best;
                    path_cell{neighbor} = [path_cell{n_best}; neighbor];
                end
            elseif test == 0
                priority_list(neighbor) = priority;
                open_list = pq_set(open_list, neighbor, priority);
                back_pointer(neighbor) = n_best;
                path_cell{neighbor} = [path_cell{n_best}; neighbor];
            end
        end
    end
end

% after finishing search process, get an optimal path from solution
path = get_path(back_pointer, map, goal);
path = path_cell{625};
[x, y, dx, dy] = dynamic_state_from_index(map, path);
path = [x, y, dx, dy];




maze1 = read_map_for_dynamics('maze1.pgm');

tic;

% determine start and goal nodes in map
[start_node1, num_nodes] = get_start_dynamic(maze1); % index = 1; 
goal_node1 = get_goal_dynamic(maze1); % index = 625; 
priority_list1 = inf * ones(num_nodes, 1);
% initialize a prior queue as open_list
% initialize a closed list for explored nodes
open_list1 = pq_init(10000); % frontier nodes
closed_list1 = []; % explored nodes

%total_search = [];
path1 = {}; % where nodes along optimal path will be stored
path1{1} = start_node1; % initialize cell array

% add start node to open list
priority_list1(start_node1) = get_h_dynamic(maze1, start_node1, goal_node1);
open_list1 = pq_insert(open_list1, start_node1, 0);

while true
    [open_list1, current1] = pq_pop(open_list1); % pop current node from pq; highest priority
    successors1 = get_neighbors_dynamic(maze1, current1); % expand all neighboring nodes from current node
    closed_list1 = [closed_list1 current1]; % push explored (current) node into the closed list
    
    if current1 == goal_node1 % if goal node is reached, stop searching
        break
    end
    
    for s1 = 1:size(successors1) % max(s) = 4
        if ismember(successors1(s1), closed_list1) == 1 % avoid reevaluating already expanded node
            continue
        else
            g1 = get_g_dynamic(maze1, start_node1, goal_node1, current1, priority_list1); %+ 1; % + get_g_dynamic(maze1, current1, successors1(s1)); % cost from start node to successor; operating cost function
            n1 = 1;
            h1 = get_h_dynamic(maze1, successors1(s1), goal_node1); % cost from successor to goal node; heuristic function
            priority1 = g1 + n1 + h1; % f cost evaluation function of successors
            
            test1 = pq_test(open_list1, successors1(s1)); % test if successor is in the open list
            if test1 == 1 % already in the open list
                if priority1 < pq_priority(open_list1, successors1(s1))
                    priority_list1(successors1(s1)) = priority1;
                    open_list1 = pq_set(open_list1, successors1(s1), priority1); % reset a successor's priority to the higher one (lower number)
                    path1{successors1(s1)} = [path1{current1}; successors1(s1)]; % record nodes of optimal path by storing where you came from
                    %total_search(end+1,:) = [successors(s), current];
                end
                
            else test1 = 0; % not in the open list
                priority_list1(successors1(s1)) = priority1;
                open_list1 = pq_insert(open_list1, successors1(s1), priority1); % insert newly discovered node in open list
                path1{successors1(s1)} = [path1{current1}; successors1(s1)]; % record nodes of optimal path by storing where you came from
                %total_search(end+1,:) = [successors(s), current];
            end
        end
    end
end

% extract state from indices of nodes along optimal path
path1 = path1{625};
X1 = [];
Y1 = [];
DX1 = [];
DY1 = [];
[x1, y1, dx1, dy1] = dynamic_state_from_index(maze1, path1);
X1 = [X1; x1];
Y1 = [Y1; y1];
DX1 = [DX1; dx1];
DY1 = [DY1; dy1];
optimal_path1 = [X1 Y1 DX1 DY1];



figure(1);
subplot(1,2,1);
plot_path(maze1, path, 'A* Search for maze1');
hold on;
subplot(1,2,2);
plot_path(maze1, optimal_path1, 'A* Search for maze1');


function cost_to_go = get_g_dynamic(map, start, goal, current, priority_list)

[x, y] = dynamic_state_from_index(map, start);
[gx, gy] = dynamic_state_from_index(map, current);

cost_to_go = abs(x - gx) + abs(y - gy); % manhattan distance (actual distance)
cost_to_go = priority_list(current) - get_h_dynamic(map, current, goal);
end

function cost_to_come = get_h_dynamic(map, successor, goal)

[x, y, dx, dy] = dynamic_state_from_index(map, goal);
[hx, hy, dhx, dhy] = dynamic_state_from_index(map, successor);

cost_to_come = (sqrt((x - hx)^2 + (y - hy)^2)); %+ (dx - dhx)^2 + (dy - dhy)^2); % euclidean distance

end



function [path] = get_path(back_pointer, map, goal)
    % initialization
    [x_g, y_g, ~, ~] = dynamic_state_from_index(map, goal);
    path = [x_g, y_g];
    n_best = back_pointer(goal);
    while true
        [x_best, y_best, ~, ~] = dynamic_state_from_index(map, n_best);
        path = [path; x_best, y_best];
        n_best = back_pointer(n_best);
        if n_best == -1
            break
        end
    end
end

% function to calculate actual cost of n_best
function [cost_g] = g(n_best, priority_list, map, h_type)
    cost_g = priority_list(n_best) - h(n_best, map, h_type);
end

% function to calculate actual cost from n_best to neighbor
function [cost_n] = cost(n_best, neighbor, map)
    [x_best, y_best, ~, ~] = dynamic_state_from_index(map, n_best);
    [x_neig, y_neig, ~, ~] = dynamic_state_from_index(map, neighbor);
    fprintf()
    cost_n = abs(x_neig - x_best) + abs(y_neig - y_best);
end

% function to calculate heuristic cost of the neighbor
function [cost_h] = h(neighbor, map, h_type)
    [x_neig, y_neig, ~] = dynamic_state_from_index(map, neighbor);
    [x_goal, y_goal, ~] = dynamic_state_from_index(map, get_goal(map));
    if h_type == 'm'
        % manhattan distance
        cost_h = abs(x_neig - x_goal) + abs(y_neig - y_goal);
    elseif h_type == 'e'
        % euclidian distance
        cost_h = sqrt((x_neig - x_goal)^2 + (y_neig - y_goal)^2);
    else
        disp('select a type of heuristic function!')
    end
end