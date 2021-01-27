% Author: Masafumi Endo
% Date: 01/14/2021
% Version: 1.0
% Description: Main file to run A* algorithm in 2D environment

% select maze to find path
maze_num = 2;
map = read_map('maze' + string(maze_num) + '.pgm');

% set start and goal info.
[start, num_nodes] = get_start(map);
goal = get_goal(map);
h_type = "e"; % select type of heuristic "m" is manhattan "e" is euclidian

@Astar.m;
astar = Astar(start, goal, map, h_type);

path = astar.search();
if h_type == 'm'
    plot_path(map, path, 'A* for 2D maze' + string(maze_num))
else
    plot_path(map, path, 'A* for 2D maze' + string(maze_num))
end

abs_path = pwd;
abs_path = strcat(abs_path, '/HW1/Astar/fig/Astar_2D_maze', string(maze_num), '_', h_type, '.png');
saveas(gcf, abs_path)