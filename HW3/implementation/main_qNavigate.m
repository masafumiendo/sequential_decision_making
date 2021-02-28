% Author: Masafumi Endo
% Date: 02/26/2021
% Objective: main function of MDP solver for step 1

% init variable
discount = 0.9;
noise = 0.1;
% load maze
maze = load_maze('maze0.txt');

% run value iteration
reward = qNavigate(maze, noise, discount);