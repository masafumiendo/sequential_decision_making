function reward = qNavigate(maze, noise, discount)
    
    % get starting index w/ number of states
    [s_start, num_state] = get_start(maze);
    num_action = 4;
    
    % init Q-function (value function w/ policy)
    Q = zeros(num_state, num_action);
    Q_new = zeros(num_state, num_action);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%  value iteration
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % init error
    error = inf;
    while error > epsilon
        % update value function for all state action pairs
        for s=1:num_state
            Q_tmp = [];
            for a=1:num_action
                v_exp = 0;
                probs = get_transition_probs(noise);
                % calculate expected value
                for idx_next=1:length(s_nexts)
                    v_exp = v_exp + probs(idx_next) * (rewards(idx_next) + discount * sum(Q(s_nexts(idx_next), :)));
                end
                Q_tmp = [Q_tmp, v_exp];
            end
            % calculate Bellman error
            error = min([error, abs(max(Q_tmp) - sum(Q(s, :)))]);
            % update Q function
            Q_new(s, :) = Q_tmp;
        end
        Q = Q_new;
    end

end

function probs = get_transition_probs(noise)
    
end