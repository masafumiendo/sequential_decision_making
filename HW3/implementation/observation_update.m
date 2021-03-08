function belief = observation_update(obs, belief, s_agent, num_states_target)
    % obs = 1: can capture target, obs = 0: can't do so
    if obs == 1
        belief = zeros(num_states_target, 1);
        belief(s_agent) = 1;
    else
        belief(s_agent) = 0;
        belief = belief / sum(belief);
    end
end