% Author: Masafumi Endo
% Date: 01/18/2021
% Version: 2.0
% Description: Implement RRT

classdef RRT
    
    % set properties
    properties
        start
        goal
        map
        dist
        sampling_rate
        min_rand
        max_rand
    end
    
    methods
        % constructor
        function obj = RRT(start, goal, map, dist, sampling_rate, rand_range)
            obj.start = start;
            obj.goal = goal;
            obj.map = map;
            obj.dist = dist; % dist to steer sampled point 
            obj.sampling_rate = sampling_rate; % rate to sample goal point
            obj.min_rand = rand_range(1);
            obj.max_rand = rand_range(2);
        end
        
        function [path] = search(obj)
            
            % initialization
            [x_start, y_start] = state_from_index(obj.map, obj.start);
            [x_goal, y_goal] = state_from_index(obj.map, obj.goal);
            node_list = [x_start, y_start];
            parent_list = [nan];
            node_cnt = 1;
            while true
                % random sampling
                [x_rand, y_rand] = get_node(obj);
                
                % find nearest node
                index_nearest = knnsearch(node_list, [x_rand, y_rand]);
                x_near = node_list(index_nearest, 1);
                y_near = node_list(index_nearest, 2);
                                
                % expand tree towards [x_near, y_near]
                [x_new, y_new] = steer(obj, [x_near, y_near], [x_rand, y_rand]);
                
                % check collision
                dx = x_new - x_near;
                dy = y_new - y_near;
                if ~check_hit(obj.map, x_near, y_near, dx, dy)
                    % add new node
                    node_list = [node_list; x_new, y_new];
                    parent_list = [parent_list; index_nearest];
                    node_cnt = node_cnt + 1;
                end
                
                % check goal
                if abs(x_goal - x_new) < 1 && abs(y_goal - y_new) < 1
                    break
                elseif node_cnt >= 1e+4 % max iteration is 10000
                    disp('error: RRT could not reach goal within 10000 nodes')
                    break
                end
            end
            
            % after finishing search process, get a path from solution
            path = get_path(obj, node_list, parent_list);
        end
        
        % function to get node
        function [x_rand, y_rand] = get_node(obj)
            if rand(1)> obj.sampling_rate
                x_rand = (obj.max_rand - obj.min_rand) * rand(1, 1) + obj.min_rand;
                y_rand = (obj.max_rand - obj.min_rand) * rand(1, 1) + obj.min_rand;
            else
                [x_goal, y_goal] = state_from_index(obj.map, obj.goal);
                x_rand = x_goal;
                y_rand = y_goal;
            end
        end
        
        % function to steer towards new node
        function [x_new, y_new] = steer(obj, node_near, node_rand)
            grad = atan2(node_rand(2) - node_near(2), node_rand(1) - node_near(1));
            dx = cos(grad) * obj.dist;
            dy = sin(grad) * obj.dist;
            x_new = node_near(1) + dx;
            y_new = node_near(2) + dy;
        end
        
        % function to get a final path from goal to start
        function [path] = get_path(obj, node_list, parent_list)
            
            % initialization
            path = [];
            index_node = length(node_list);
            while true
                path = [path; node_list(index_node, :)];
                index_node = parent_list(index_node, :);
                if isnan(index_node)
                    break
                end
            end
        end
    end
end