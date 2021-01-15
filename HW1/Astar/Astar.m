% Author: Masafumi Endo
% Date: 01/14/2021
% Version: 1.0
% Description: Implement A*

classdef Astar
    
    properties
    end
    
    methods
        function [] = search()
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
                    eval = actual(n_best, neighbor) + heuristic(map, neighbor);
                    % check the neighbor is in open_list or not
                    if pq_test(open_list, neighbor)
                        % update evaluation list
                        if eval < eval_list(neighbor)
                            eval_list(neighbor) = eval;
                        end
                    else
                        % add neighbor to open_list
                        open_list = pq_insert(open_list, neighbor, eval);
                    end
                end
            end
        end
        
        function [] = get_path()
            
        end
        
        function [] = actual(n_best, neighbor)
            
        end
        
        function [] = heuristic(map, neighbor)
            
        end
    end
end