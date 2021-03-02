% function to perform value iteration until converge
function Q = value_iteration(Tfunc, Rfunc, num_states, num_actions, discount, epsilon, s_start, maze)
    % init Q-function
    Q_new = zeros(num_states, num_actions);
    while true
        Q_cur = Q_new;
        error = 0;
        for s=1:num_states
            for a=1:num_actions
                r_cur = Rfunc(s); % current reward
                v_exp = reshape(Tfunc(s, a, :), [1, num_states]) * max(Q_cur, [], 2); % expected values
                Q_new(s, a) = r_cur + discount * v_exp;
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