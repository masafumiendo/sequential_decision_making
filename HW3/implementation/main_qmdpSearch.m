% Author: Masafumi Endo
% Date: 02/28/2021
% Objective: run qmdpSearch

% set variable
noise = 0.1;
discount = 0.9;
epsilon = 0.001;
% load maze environment
maze = load_maze('maze1.txt');

reward = qmdpSearch(maze, noise, discount, epsilon);