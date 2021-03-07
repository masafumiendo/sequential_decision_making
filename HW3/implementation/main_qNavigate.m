% Author: Masafumi Endo
% Date: 02/28/2021
% Objective: run qNavigate

% set variable
noise = 0.2;
discount = 0.9;
epsilon = 0.00001;
% load maze environment
maze = load_maze('maze0.txt');

reward = qNavigate(maze, noise, discount, epsilon);
