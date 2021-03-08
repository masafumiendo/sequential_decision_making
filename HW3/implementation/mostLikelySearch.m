% Author: Masafumi Endo
% Date: 02/28/2021
% Objective: implement qmdpSearch to solve POMDP

function reward = mostLikelySearch(maze, noise, discount, epsilon)

    % get starting index w/ num of states
    [s_start, num_states] = get_start(maze);
    num_states_target = num_states;
    num_states = num_states^2; % (agent * target) state
    num_actions = 4; % action space
    
    % get transition probability function and reward function
    Tfunc = get_trans_func(num_states, num_actions, noise, maze);
    Rfunc = get_reward_func(num_states, num_actions);
    
    % draw maze environment
    draw_maze(maze, s_start);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% value iteration
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Q = value_iteration(Tfunc, Rfunc, num_states, num_actions, discount, epsilon);
    
    % show Q-function
    draw_maze(maze, s_start, max(Q, [], 2))
    
    % init reward
    reward = get_reward(maze, s_start);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% run mostLikelySearch for 100 iterations
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % init belief state of target
    belief = ones(num_states_target, 1) / num_states_target;
    belief(6, 1) = 0; % obstacle
    belief = belief / sum(belief);
    % set initial position
    s_agent = s_start;
    for iter=1:100
        % move target
        maze = moveTarget(maze);
        % retrieve most-likely target position from belief state
        [~, s_target] = max(belief);
        s_target = (s_agent - 1) * num_states_target + s_target;
        % choose best action based on most-likely position of the target
        [~, a] = max(Q(s_target, :));
        % take action to move
        s_agent = move_maze(maze, s_agent, a, noise);
        % observe environment
        obs = getObservation(maze, s_agent);
        % belief update
        belief = observation_update(obs, belief, s_agent, num_states_target);
        
        % show robot moving process
        draw_maze(maze, s_agent, max(Q, [], 2))
        % get reward
        reward = reward + get_reward(maze, s_agent);
    end
    
end