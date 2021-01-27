% Author: Masafumi Endo
% Date: 01/26/2021
% Version: 2.0
% Description: Main file to run greedy-A* algorithm in 2D environment

% select maze to find path
map = read_map('maze2.pgm');

% set start and goal info.
[start, num_nodes] = get_start(map);
goal = get_goal(map);
h_type = "e"; % select type of heuristic "m" is manhattan "e" is euclidian
epsilon = 10;
t_limit = 1; % 0.05, 0.25, or 1 sec

@greedy_Astar.m;
greedy_astar = greedy_Astar(start, goal, map, h_type, epsilon, t_limit);

path = greedy_astar.greedy_search();
if h_type == 'm'
    plot_path(map, path, 'A* path planning result, Manhattan heuristic')
else
    plot_path(map, path, 'A* path planning result, Euclidian heuristic')
end