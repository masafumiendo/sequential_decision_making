% Author: Masafumi Endo
% Date: 01/14/2021
% Version: 1.0
% Description: Implement A*

classdef Astar
    
    % set properties
    properties
        start
        goal
        map
        h_type
    end
    
    methods
        % constructor
        function obj = Astar(start, goal, map, h_type)
            obj.start = start;
            obj.goal = goal;
            obj.map = map;
            obj.h_type = h_type;
        end
        
        function [path] = search(obj)
            
            % initialization
            [~, num_nodes] = get_start(obj.map);
            eval_list = inf * ones(num_nodes, 1); % evaluation function list
            open_list = pq_init(num_nodes); % open list
            % starting position
            eval_list(obj.start) = 0;
            open_list = pq_set(open_list, obj.start, eval_list(obj.start));
            while true
                % pick best node and remove it from open_list
                [open_list, n_best] = pq_pop(open_list);
                % check n_best is goal or not
                if n_best == obj.goal
                    break
                end
                
                % expand all nodes that are neibhbors of n_best
                % note that the last action is [0; 0] so the agent doesn't
                % move
                [neighbors, ~] = get_neighbors(obj.map, n_best);
                % -1 is for removing the last action 
                for index_neighbor=1:length(neighbors)-1
                    neighbor = neighbors(index_neighbor);
                    eval = actual(obj, n_best, eval_list) + heuristic(obj, neighbor);
                    % check the current and previous cost
                    if eval < eval_list(neighbor)
                        eval_list(neighbor) = eval;
                        % update open list
                        open_list = pq_set(open_list, neighbor, eval);
                    end
                end
            end
            
            % after finishing search process, get an optimal path from solution
            path = get_path(obj, eval_list);
        end
        
        function [path] = get_path(obj, eval_list)
            % initialization
            path = [];
            n_best = obj.goal; % start from the goal node
            while n_best ~= obj.start
                [x_best, y_best] = state_from_index(obj.map, n_best);
                path = [path; x_best, y_best];
                [neighbors, ~] = get_neighbors(obj.map, n_best);
                neighbors = neighbors(1:end-1);
                [~, index_neighbor] = min(eval_list(neighbors));
                % update n_best for the next loop
                n_best = neighbors(index_neighbor);
            end
        end
        
        % function to calculate actual cost of the neighbor
        function [cost_a] = actual(obj, n_best, eval_list)
            cost_a = eval_list(n_best) + 1; % cost-to-come plus 1
        end
        
        % function to calculate heuristic cost of the neighbor
        function [cost_h] = heuristic(obj, neighbor)
            [x_neig, y_neig] = state_from_index(obj.map, neighbor);
            [x_goal, y_goal] = state_from_index(obj.map, get_goal(obj.map));
            if obj.h_type == 'm'
                % manhattan distance
                cost_h = abs(x_neig - x_goal) + abs(y_neig - y_goal);
            elseif obj.h_type == 'e'
                % euclidian distance
                cost_h = sqrt((x_neig - x_goal)^2 + (y_neig - y_goal)^2);
            else
                disp('select a type of heuristic function!')
            end
        end
    end
end