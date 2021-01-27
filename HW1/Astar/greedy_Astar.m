% Author: Masafumi Endo
% Date: 01/26/2021
% Version: 2.0
% Description: Implement greedy-A*

classdef greedy_Astar
    
    % set properties
    properties
        start
        goal
        map
        h_type
        epsilon
        t_limit
    end
    
    methods
        % constructor
        function obj = greedy_Astar(start, goal, map, h_type, epsilon, t_limit)
            obj.start = start;
            obj.goal = goal;
            obj.map = map;
            obj.h_type = h_type;
            obj.epsilon = epsilon;
            obj.t_limit = t_limit;
        end
        
        function [path] = greedy_search(obj)
            tic
            while toc < obj.t_limit
                [path, num_node] = search(obj);
                disp(' ')
                disp('-------------------')
                fprintf('epsilon: %f \n', obj.epsilon)
                fprintf('number of nodes expanded: %d, path length: %f \n', num_node, length(path))
                disp('-------------------')
                disp(' ')
                
                obj.epsilon = obj.epsilon - 0.5 * (obj.epsilon - 1);
                if obj.epsilon == 1
                    break
                end
                if obj.epsilon < 1.001
                    obj.epsilon = 1;
                end
            end
        end
        
        function [path, num_node] = search(obj)
            
            % initialization
            [~, num_nodes] = get_start(obj.map);
            priority_list = inf * ones(num_nodes, 1); % list containing priorities
            open_list = pq_init(1e+4); % open list
            closed_list = []; % closed list
            back_pointer = nan * ones(num_nodes, 1); % back-pointer attribute
            % starting position
            priority_list(obj.start) = obj.epsilon * h(obj, obj.start);
            open_list = pq_set(open_list, obj.start, priority_list(obj.start));
            back_pointer(obj.start) = -1;
            num_node = 1;
            while true
                % pick best node and remove it from open_list
                [open_list, n_best] = pq_pop(open_list);
                num_node = num_node + 1;
                % add best node to closed_list
                closed_list = [closed_list; n_best];

                % check n_best is goal or not
                if n_best == obj.goal
                    break
                end
                
                % expand all nodes that are neibhbors of n_best
                [neighbors, ~] = get_neighbors(obj.map, n_best);
                neighbors = neighbors((neighbors == n_best) ~= 1);
                for index_neighbor=1:length(neighbors)
                    neighbor = neighbors(index_neighbor);
                    % calculate each cost
                    cost_g = g(obj, n_best, priority_list);
                    cost_n = cost(obj, n_best, neighbor);
                    cost_h = obj.epsilon * h(obj, neighbor);
                    priority = cost_g + cost_n + cost_h;
                    % neighbor is in open_list
                    if pq_test(open_list, neighbor)
                        if priority < priority_list(neighbor)
                            % update open_list
                            priority_list(neighbor) = priority;
                            open_list = pq_set(open_list, neighbor, priority);
                            back_pointer(neighbor) = n_best;
                        end
                    % neighbor is in closed_list
                    elseif ismember(neighbor, closed_list)
                        continue
%                         if priority < priority_list(neighbor)
%                             % remove neighbor from closed_list
%                             index = closed_list(:) == neighbor;
%                             closed_list = closed_list(~index);
%                             % add it to open_list
%                             priority_list(neighbor) = priority;
%                             open_list = pq_set(open_list, neighbor, priority);
%                             back_pointer(neighbor) = n_best;
%                         end
                    % neighbor is not in both open_list and closed_list 
                    else
                        % add neighbor to open_list
                        priority_list(neighbor) = priority;
                        open_list = pq_set(open_list, neighbor, priority);
                        back_pointer(neighbor) = n_best;
                    end
                end
            end

            % after finishing search process, get an optimal path from solution
            path = get_path(obj, back_pointer);
        end
        
        function [path] = get_path(obj, back_pointer)
            % initialization
            [x_g, y_g] = state_from_index(obj.map, obj.goal);
            path = [x_g, y_g];
            n_best = back_pointer(obj.goal);
            while true
                [x_best, y_best] = state_from_index(obj.map, n_best);
                path = [path; x_best, y_best];
                n_best = back_pointer(n_best);
                if n_best == -1
                    break
                end
            end
        end
        
        % function to calculate actual cost of n_best
        function [cost_g] = g(obj, n_best, priority_list)
            cost_g = priority_list(n_best) - obj.epsilon * h(obj, n_best);
        end
        
        % function to calculate actual cost from n_best to neighbor
        function [cost_n] = cost(obj, n_best, neighbor)
            [x_best, y_best] = state_from_index(obj.map, n_best);
            [x_neig, y_neig] = state_from_index(obj.map, neighbor);
            cost_n = abs(x_neig - x_best) + abs(y_neig - y_best);
        end
        
        % function to calculate heuristic cost of the neighbor
        function [cost_h] = h(obj, neighbor)
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