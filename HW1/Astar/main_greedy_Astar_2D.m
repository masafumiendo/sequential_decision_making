% Author: Masafumi Endo
% Date: 01/26/2021
% Version: 2.0
% Description: Main file to run greedy-A* algorithm in 2D environment

% select maze to find path
maze_num = 2;
map = read_map('maze' + string(maze_num) + '.pgm');

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
    plot_path(map, path, 'Greedy-A* for 2D maze' + string(maze_num) + ' in ' + string(t_limit) + ' sec')
else
    plot_path(map, path, 'Greedy-A* for 2D maze' + string(maze_num) + ' in ' + string(t_limit) + ' sec')
end

abs_path = pwd;
abs_path = strcat(abs_path, '/HW1/Astar/fig/greedy_Astar_2D_maze', string(maze_num), '_', h_type, '_t_limit_', string(t_limit), '.png');
saveas(gcf, abs_path)