% Author: Masafumi Endo
% Date: 01/18/2021
% Version: 1.0
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
            node_list = [x_start, y_start;];
            while true
                % random sampling
                [x_rand, y_rand] = get_node(obj);
                
                % find nearest node
                index_nearest = knnsearch(node_list, [x_rand, y_rand]);
                x_near = node_list(index_nearest, 1);
                y_near = node_list(index_nearest, 2);
                                
                % expand tree towards [x_near, y_near]
                [x_new, y_new] = steer([x_near, y_near], [x_rand, y_rand]);
                
                % check collision
                dx = x_new - x_near;
                dy = y_new - y_near;
                if ~check_hit(obj.map, x_near, y_near, dx, dy)
                    % add new node
                    node_list = [node_list; x_new, y_new];
                end
                
                % check goal
                if abs(x_goal - x_new) < 1 && abs(y_goal - y_new) < 1
                    break
                end
            end
            
            % after finishing search process, get a path from solution
            path = get_path();
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
        function [x_new, y_new] = steer(node_near, node_rand)
            
        end
        
        % function to get a path
        function [path] = get_path(obj)
            
        end
    end
end