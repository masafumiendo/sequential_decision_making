% Constants %
map = read_map_for_dynamics('maze2.pgm');
move_cost = 1;

% Initialization %
goal = get_goal_dynamic(map);
[start, num_nodes] = get_start_dynamic(map);
current_location = start;
maxV = map.maxV;
initial_cost = 20000;
cost_all = initial_cost*ones(num_nodes,1);
cost_all(start) = 0;
goal_flag = 0;
open_list = pq_init(1000);
open_list = pq_insert(open_list, start, cost_all(start));
iter = 0;
final_path = [];
path_array = cell(length(cost_all), 1);
path_array{start} = start;

while goal_flag ~= 1
    iter = iter + 1;
    if current_location == goal
        goal_flag = 1;
        break
    end
    
    try
        [open_list, current_location] = pq_pop(open_list);
    catch error
        hello = 1;
    end
    [neighbors, num_neighbors] = get_neighbors_dynamic(map, current_location);
    neighbors = neighbors((neighbors == current_location)~=1); %removing self from neighbors
    current_cost = cost_all(current_location);
    %if a location has no neighbors (a corner) then assign a high cost
    if isempty(neighbors)
        cost_all(current_location) = -initial_cost;
    end
    %last value of neighbors is the current location
    %assign neighbors a new cost = move_cost + hueristic
    for i = 1:length(neighbors)
        neighbor_cost = current_cost + move_cost + Hueristic_Dynamic(neighbors(i), map);
        if neighbor_cost < cost_all(neighbors(i))
           disp('Location Being Checked: ' + string(neighbors(i)) + ', Old Cost: ' + string(cost_all(neighbors(i))) + ', New Cost: ' + string(neighbor_cost))
           cost_all(neighbors(i)) = neighbor_cost; 
           if pq_test(open_list, neighbors(i))
               [x, y, dx, dy] = dynamic_state_from_index(neighbors(i));
               disp('Point already in Queue')
               disp('ID' + string(neighbors(i)) + ' , ' + string(x) + ' ' + string(y))
           end
           open_list = pq_insert(open_list, neighbors(i), neighbor_cost);
           path_array{neighbors(i)} = [neighbors(i); path_array{current_location}];
        else
            disp('Location Being Checked: ' + string(neighbors(i)) + ' had no new neighbors')
        end
    end
    

end
% need to find path from goal to start
% current_location = goal;
% while all(current_location ~= start)
%     [X, Y, dx, dy] = dynamic_state_from_index(map, current_location);
% %     final_path = [final_path; X,Y];
%     neighbors = get_neighbors_dynamic(map, current_location);
%     neighbors = neighbors((neighbors == current_location)~=1); %removing self from neighbors
%     if isempty(neighbors)
%         hello = 1;
%     end
%     [next_val, next_ind] = min(abs(cost_all(neighbors)));
%     next_loc = neighbors(next_ind);
%     current_location = next_loc;
%     
% end
for i = 1:length(path_array{goal})
    [x,y] = dynamic_state_from_index(map, path_array{goal}(i));
   final_path = [final_path; [x, y]];
end
plot_path(map, final_path, 'A* with ManhattanPlus Distance Hueristic')


function est = Hueristic_Dynamic(L1, map)
[X, Y, DX, DY] = dynamic_state_from_index(map, L1);
[X_goal, Y_goal, DX_goal, DY_goal] = dynamic_state_from_index(map, get_goal(map));

% est = euclidean([X,Y], [X_goal, Y_goal]);
% est = manhattan([X,Y], [X_goal, Y_goal]);
est = manhattan_dynamic([X,Y], [DX, DY], [X_goal, Y_goal]);
% est = euclidean_dynamic([X,Y], [DX, DY], [X_goal, Y_goal]);
end

function d = manhattan_dynamic(L1, V1, goal)
    %incetive to go as fast as possible
    d = manhattan(L1, goal)/norm([2,2]) - sum(V1);
    d = max(0, d); %make sure hueristic can't be negative so weird stuff doens't start to happen
    
end

function d = euclidean_dynamic(L1, V1, goal)
% max = ((25 - 1)^2 + (25 - 1)^2)^(0.5) - (2 + 2) = 29.48
%incetive to go as fast as possible
    d = euclidean(L1, goal)/norm([2,2]) - sum(V1);
    d = max(0, d); %make sure hueristic can't be negative so weird stuff doens't start to happen
    
end

function d = euclidean(L1, goal)
d = norm(L1 - goal);
end

function d = manhattan(L1, goal)
    d = sum(abs(L1-goal));
end