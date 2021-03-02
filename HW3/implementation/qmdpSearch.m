% Author: Masafumi Endo
% Date: 02/28/2021
% Objective: implement qmdpSearch to solve POMDP

function reward = qmdpSearch(maze, noise, discount, epsilon)

    % get starting index w/ num of states
    [s_start, num_states] = get_start(maze);
    num_actions = 4; % action space
    
    % get transition probability function and reward function
    Tfunc = get_trans_func(num_states, num_actions, noise, maze);
    Rfunc = get_reward_func(num_states, num_actions);
    
    % draw maze environment
    draw_maze(maze, s_start);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% value iteration
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Q_new = value_iteration(Tfunc, Rfunc, num_states, num_actions, discount, epsilon, s_start);
    
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

% function to perform value iteration until converge
function Q = value_iteration(Tfunc, Rfunc, num_states, num_actions, discount, epsilon, s_start)
    % init Q-function
    Q_new = zeros(num_states, num_actions);
    while true
        Q_cur = Q_new;
        error = 0;
        for s=1:num_states
            for a=1:num_actions
                r_cur = Rfunc(s); % current reward
                r_exp = reshape(Tfunc(s, a, :), [1, num_states]) * max(Q_cur, [], 2); % expected values
                Q_new(s, a) = r_cur + discount * r_exp;
            end
        end
        % update Bellman error if necessary
        error = max([error, abs(max(Q_cur(s, :)) - max(Q_new(s, :)))]);
        
        % show value iteration process
        draw_maze(maze, s_start, max(Q_new, [], 2))
        
        % finish value iteration 
        if error < epsilon
            break
        end
    end
    % finalize Q-function
    Q = Q_new;
end

% function to construct transition probability function
function Tfunc = get_trans_func(num_states, num_actions, noise, maze)
    % init function
    Tfunc = zeros(num_states, num_actions, num_states); % T(s, a, s')
    for s_agent=1:sqrt(num_states)
        for s_target=1:sqrt(num_states)
            % re-construct state from (s_agent, s_target)
            s = sqrt(num_states) * (s_agent - 1) + s_target;
            for a=1:num_actions
                sn_agent = move_maze(maze, s_agent, a, 0);
                % calculate transition probability when correct movement
                for a_target=1:num_actions
                    sn_target = move_maze(maze, s_target, a_target, 0);
                    % re-construct state from (sn_agent, sn_target)
                    s_next = sqrt(num_states) * (sn_agent - 1) + sn_target;
                    Tfunc(s, a, s_next) = Tfunc(s, a, s_next) + (1 - noise) / num_actions;
                end
                % calculate transition probability when incorrect movement
                for a_=1:num_actions
                    sn_agent_ = move_maze(maze, s_agent, a_, 0);
                    for a_target_=1:num_actions
                        sn_target_ = move_maze(maze, s_target, a_target_, 0);
                        s_next_ = sqrt(num_states) * (sn_agent_ - 1) + sn_target_;
                        Tfunc(s, a_, s_next_) = Tfunc(s, a_, s_next_) + noise / num_actions^2;
                    end
                end
            end
        end        
    end
end

% function to construct reward function
function Rfunc = get_reward_func(num_states, num_actions)
    % init function
    Rfunc = zeros(num_states, 1);    
    for s_agent=1:sqrt(num_states)
        for s_target=1:sqrt(num_states)
            % re-construct state from (s_agent, s_target)
            s = sqrt(num_states) * (s_agent - 1) + s_target;
            for a_agent=1:num_actions
                % if positional states same, update reward function
                if s_agent == s_target
                    Rfunc(s) = 1;
                end
            end
        end
    end
end