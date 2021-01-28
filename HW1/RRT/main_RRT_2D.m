% Author: Masafumi Endo
% Date: 01/19/2021
% Version: 1.0
% Description: Main file to run RRT algorithm in 2D environment

% select maze to find path
maze_num = 2;
map = read_map('maze' + string(maze_num) + '.pgm');

% set start and goal info.
[start, num_nodes] = get_start(map);
goal = get_goal(map);
dist = 1; % dist to steer sampled point
sampling_rate = 0.2; % rate to sample goal point
rand_range = [0, 25]; % range to sample random point [min, max]

@RRT.m;
rrt = RRT(start, goal, map, dist, sampling_rate, rand_range);
[path, runtime] = rrt.search();
path_length = get_path_length(path);
fprintf('runtime: %f, path length: %f \n', runtime, path_length)

plot_path(map, path, 'RRT for 2D maze' + string(maze_num) + ', length: ' + string(path_length) + ', time: ' + sprintf('%.2f', runtime) + ' sec')

abs_path = pwd;
abs_path = strcat(abs_path, '/HW1/RRT/fig/RRT_2D_maze', string(maze_num), '.png');
saveas(gcf, abs_path)