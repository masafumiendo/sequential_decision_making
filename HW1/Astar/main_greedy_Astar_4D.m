% Author: Masafumi Endo
% Date: 01/27/2021
% Version: 1.0
% Description: Main file to run greedy-A* algorithm in 4D environment

% select maze to find path
maze_num = 2;
map = read_map_for_dynamics('maze' + string(maze_num) + '.pgm');

% set start and goal info.
[start, num_nodes] = get_start_dynamic(map);
goal = get_goal_dynamic(map);
h_type = "e"; % select type of heuristic "m" is manhattan "e" is euclidian
epsilon = 10;
t_limit = 1; % 0.05, 0.25, or 1 sec

@greedy_Astar_dynamic.m;
greedy_astar_dynamic = greedy_Astar_dynamic(start, goal, map, h_type, epsilon, t_limit);

path = greedy_astar_dynamic.greedy_search();
if h_type == 'm'
    plot_path(map, path, 'Greedy-A* for 4D maze' + string(maze_num) + ' in ' + string(t_limit) + ' sec')
else
    plot_path(map, path, 'Greedy-A* for 4D maze' + string(maze_num) + ' in ' + string(t_limit) + ' sec')
end

abs_path = pwd;
abs_path = strcat(abs_path, '/HW1/Astar/fig/greedy_Astar_4D_maze', string(maze_num), '_', h_type, '_t_limit_', string(t_limit), '.png');
saveas(gcf, abs_path)