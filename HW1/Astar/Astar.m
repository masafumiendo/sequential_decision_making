% Author: Masafumi Endo
% Date: 01/14/2021
% Version: 1.0
% Description: Implement A*

classdef Astar
    
    properties
    end
    
    methods
        function [] = search(start, goal, map, h_type)
            
            % initialization
            [~, num_nodes] = get_start(map);
            eval_list = inf * ones(num_nodes, 1); % evaluation function list
            open_list = pq_init(num_nodes); % open list
            % starting position
            eval_list(start) = 0;
            open_list = pq_set(open_list, start, eval_list(start));
            while true
                % pick best node and remove it from open_list
                [open_list, n_best] = pq_pop(open_list);
                
                % check n_best is goal or not
                if n_best == goal
                    break
                end
                
                % expand all nodes that are neibhbors of n_best
                [neighbors, ~] = get_neighbors(map, n_best);
                for index_neighbor=1:length(neighbors)
                    neighbor = neighbors(index_neighbor);
                    eval = actual(map, n_best, neighbor, eval_list) + heuristic(map, neighbor, h_type);
                    % check the neighbor is in open_list or not
                    if pq_test(open_list, neighbor)
                        % update evaluation function and open list
                        if eval < eval_list(neighbor)
                            eval_list(neighbor) = eval;
                            open_list = pq_set(open_list, neighbor, eval);
                        end
                    else
                        % add neighbor to open_list
                        open_list = pq_set(open_list, neighbor, eval);
                    end
                end
            end
        end
        
        function [] = get_path()
            
        end
        
        % function to calculate actual cost of the neighbor
        function [cost_a] = actual(map, n_best, neighbor, eval_list)
            [x_best, y_best] = state_from_index(map, n_best);
            [x_neig, y_neig] = state_from_index(map, neighbor);
            if x_best == x_neig && y_best == y_neig
                cost_a = eval_list(n_best);
            else
                cost_a = eval_list(n_best) + 1;
            end
        end
        
        % function to calculate heuristic cost of the neighbor
        function [cost_h] = heuristic(map, neighbor, h_type)
            [x_neig, y_neig] = state_from_index(map, neighbor);
            [x_goal, y_goal] = state_from_index(map, get_goal(map));
            if h_type == 'm'
                % manhattan distance
                cost_h = abs(x_neig - x_goal) + abs(y_neig - y_goal);
            elseif h_type == 'e'
                % euclidian distance
                cost_h = sqrt((x_neig - x_goal)^2 + (y_neig - y_goal)^2);
            else
                disp('select a type of heuristic function!')
            end
        end
    end
end