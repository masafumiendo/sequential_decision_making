% Author: Masafumi Endo
% Date: 01/26/2021
% Version: 2.0
% Description: Main file to run A* algorithm in 4D environment

% select maze to find path
maze_num = 2;
map = read_map_for_dynamics('maze' + string(maze_num) + '.pgm');

% set start and goal info.
[start, num_nodes] = get_start_dynamic(map);
goal = get_goal_dynamic(map);
h_type = "e"; % select type of heuristic "m" is manhattan "e" is euclidian

@Astar_dynamic.m;
astar_dynamics = Astar_dynamic(start, goal, map, h_type);

path = astar_dynamics.search();
if h_type == 'm'
    plot_path(map, path, 'A* for 4D maze' + string(maze_num))
else
    plot_path(map, path, 'A* for 4D maze' + string(maze_num))
end

abs_path = pwd;
abs_path = strcat(abs_path, '/HW1/Astar/fig/Astar_4D_maze', string(maze_num), '_', h_type, '.png');
saveas(gcf, abs_path)