function T_bel = get_trans_belief(num_states, num_actions, maze)
    T_bel = zeros(num_states, num_states);
    for s=1:num_states
        for a=1:num_actions
            s_next = move_maze(maze, s, a, 0);
            T_bel(s, s_next) = T_bel(s, s_next) + 1 / num_actions;
        end
    end
end