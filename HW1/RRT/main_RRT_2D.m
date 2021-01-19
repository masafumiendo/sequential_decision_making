% Author: Masafumi Endo
% Date: 01/14/2021
% Version: 1.0
% Description: Main file to run RRT algorithm in 2D environment

% select maze to find path
map = read_map('maze2.pgm');

% set start and goal info.
[start, num_nodes] = get_start(map);
goal = get_goal(map);
dist = 1; % dist to steer sampled point
sampling_rate = 0.2; % rate to sample goal point
rand_range = [0, 25]; % range to sample random point

@RRT.m;
rrt = RRT(start, goal, map, dist, sampling_rate, rand_range);
path = rrt.search();
plot_path(map, path, 'RRT path planning result')