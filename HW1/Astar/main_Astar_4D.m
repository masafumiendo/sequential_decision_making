% Author: Masafumi Endo
% Date: 01/24/2021
% Version: 1.0
% Description: Main file to run A* algorithm in 4D environment

% select maze to find path
map = read_map_for_dynamics('maze2.pgm');

% set start and goal info.
[start, num_nodes] = get_start_dynamic(map);
goal = get_goal_dynamic(map);
h_type = "m"; % select type of heuristic "m" is manhattan "e" is euclidian

@Astar_dynamic.m;
astar_dynamic = Astar_dynamic(start, goal, map, h_type);

path = astar_dynamic.search();
if h_type == 'm'
    plot_path(map, path, 'A* path planning result, Manhattan heuristic')
else
    plot_path(map, path, 'A* path planning result, Euclidian heuristic')
end