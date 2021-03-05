function belief = belief_update(obs, belief, s_agent, num_states_target)
    if obs == 1
        belief = zeros(num_states_target, 1);
        belief(s_agent) = 1;
    else
        belief(s_agent) = 0;
        belief = belief / sum(belief);
    end
end