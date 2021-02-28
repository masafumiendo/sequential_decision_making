% Author: Masafumi Endo
% Date: 02/28/2021
% Objective: implement qNavigate to solve MDP

function reward = qNavigate(maze, noise, discount, epsilon)

    % get starting index w/ num of states
    [s_start, num_states] = get_start(maze);
    num_actions = 4; %
    
    % init Q-function
    Q_new = zeros(num_states, num_actions);
    
    % draw maze environment
    draw_maze(maze, s_start);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% value iteration
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    while true
        Q_cur = Q_new;
        error = 0;
        for s=1:num_states
            for a=1:num_actions
                % update Q-function
                Q_new(s, a) = get_reward(maze, s) + get_expected_values(s, a, Q_cur, discount, noise, maze);
            end
            % update Bellman error if necessary
            error = max([error, abs(max(Q_cur(s, :)) - max(Q_new(s, :)))]);
        end
        
        % show value iteration process
        draw_maze(maze, s_start, max(Q_new, [], 2))
        
        % finish value iteration 
        if error < epsilon
            break
        end
    end
    
    % init reward
    reward = get_reward(maze, s_start);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% run qNavigate for 100 iterations
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    s_cur = s_start;
    for iter=1:100
        % choose best action based on Q-function
        [~, a] = max(Q_new(s_cur, :));
        % take action to move
        s_cur = move_maze(maze, s_cur, a, noise);
        % show robot moving process
        draw_maze(maze, s_cur, max(Q_new, [], 2))
        
        % get reward
        reward = reward + get_reward(maze, s_cur);
    end
    
end

% function to calculate total expected values
function v_exp = get_expected_values(s, a, Q_cur, discount, noise, maze)
    % init expected values
    v_exp = 0;
    for transition=transitions_at(s, a, noise, maze)
        % transition(1): probability, transition(2): next state index
        v_exp = v_exp + discount * transition(1) * max(Q_cur(transition(2), :));
    end
end

% function to provide transitions (probability, corresponding next state)
function transitions = transitions_at(s, a, noise, maze)
    transitions = zeros(2, 4); % store probs w/ corresponding state indices
    for a_=1:4
        if a_ == a
            transitions(1, a_) = (1 - noise) + noise / 4;
        else
            transitions(1, a_) = noise / 4;
        end
        s_next = move_maze(maze, s, a_, 0);
        transitions(2, a_) = s_next;
    end
end