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