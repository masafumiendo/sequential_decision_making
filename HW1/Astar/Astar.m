% Author: Masafumi Endo
% Date: 01/14/2021
% Version: 1.0
% Description: Implement A*

classdef Astar
    
    properties
    end
    
    methods
        function [path] = search(obj, start, goal, map, h_type)
            
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
                for index_neighbor=1:length(neighbors)-1
                    neighbor = neighbors(index_neighbor);
                    eval = actual(obj, n_best, eval_list) + heuristic(obj, map, neighbor, h_type);
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
            
            % after finishing search process, get a path from solution
            path = get_path(obj, start, n_best, map, eval_list);
        end
        
        function [path] = get_path(obj, start, n_best, map, eval_list)
            % initialization
            path = [];
            while n_best ~= start
                [x_best, y_best] = state_from_index(map, n_best);
                path = [path; x_best, y_best];
                [neighbors, ~] = get_neighbors(map, n_best);
                neighbors = neighbors(1:end-1);
                [~, index_neighbor] = min(eval_list(neighbors));
                n_best = neighbors(index_neighbor);
            end
        end
        
        % function to calculate actual cost of the neighbor
        function [cost_a] = actual(obj, n_best, eval_list)
            cost_a = eval_list(n_best) + 1;
        end
        
        % function to calculate heuristic cost of the neighbor
        function [cost_h] = heuristic(obj, map, neighbor, h_type)
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