% Author: Masafumi Endo
% Date: 02/28/2021
% Objective: run mostLikelySearch

% set variable
noise = 0.1;
discount = 0.9;
epsilon = 0.00001;
% load maze environment
maze = load_maze('maze1.txt');

reward = mostLikelySearch(maze, noise, discount, epsilon);