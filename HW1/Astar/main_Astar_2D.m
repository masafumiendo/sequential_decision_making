% Author: Masafumi Endo
% Date: 01/14/2021
% Version: 1.0
% Description: Main file to run A* algorithm in 2D environment

% select maze to find path
map = read_map('maze2.pgm');

% set start and goal info.
[start, num_nodes] = get_start(map);
goal = get_goal(map);
h_type = "m"; % select type of heuristic "m" is manhattan "e" is euclidian

@Astar.m;
astar = Astar(start, goal, map, h_type);

path = astar.search();
if h_type == 'm'
    plot_path(map, path, 'A* path planning in 2D Space, Manhattan heuristic')
else
    plot_path(map, path, 'A* path planning in 2D Space, Euclidian heuristic')
end