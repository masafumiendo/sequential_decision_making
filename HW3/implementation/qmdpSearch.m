% Author: Masafumi Endo
% Date: 02/28/2021
% Objective: implement qmdpSearch to solve POMDP

function reward = qmdpSearch(maze, noise, discount, epsilon)

    % get starting index w/ num of states
    [s_start, num_states] = get_start(maze);
    num_states = num_states^2; 
    num_actions = 4; % action space
    
    % get transition probability function and reward function
    Tfunc = get_trans_func(num_states, num_actions, noise, maze);
    Rfunc = get_reward_func(num_states, num_actions);
    
    % draw maze environment
    draw_maze(maze, s_start);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% value iteration
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Q_new = value_iteration(Tfunc, Rfunc, num_states, num_actions, discount, epsilon, s_start, maze);
    
    % init reward
    reward = get_reward(maze, s_start);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% run qmdpSearch for 100 iterations
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    s_cur = s_start;
    for iter=1:100
        % calculate QMDP
        % choose best action based on QMDP
        [~, a] = max(Qmdp(b, :));
        % take action to move
        s_cur = move_maze(maze, s_cur, a, noise);
        
        % show robot moving process
        draw_maze(maze, s_cur, max(Q_new, [], 2))
        % get reward
        reward = reward + get_reward(maze, s_cur);
        % move target for the next loop
        maze = moveTarget(maze);
    end
    
end





