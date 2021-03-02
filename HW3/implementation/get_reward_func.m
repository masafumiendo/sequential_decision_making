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