classdef PoseNode < handle
    %POSENODE A class for pose node
    
    properties (Access = public)
        id    % Id of this pose node
        pose  % Pose of this pose node
    end  % properties public
    
    properties (Dependent = true)
        x    % X coordinate
        y    % Y coordinate
        yaw  % Yaw angle
        rt   % Transformation local to global
    end  % properties dependent
    
    methods
        
        function obj = PoseNode(id, pose)
            % Constructor of PoseNode
            obj.id   = id;
            obj.pose = pose(:);
        end
        
        function plot(obj)
            % Plot all pose nodes position
            x = [obj.x];
            y = [obj.y];
            plot(x, y, 'b');
        end
        
        function x = get.x(obj)
            x = obj.pose(1);
        end
        
        function y = get.y(obj)
            y = obj.pose(2);
        end
        
        function yaw = get.yaw(obj)
            yaw = obj.pose(3);
        end
        
        function rt = get.rt(obj)
            R = [cos(obj.yaw) -sin(obj.yaw);
                 sin(obj.yaw)  cos(obj.yaw)];
            rt = [R [obj.x; obj.y]; 0 0 1];
        end
        
    end  % methods public
    
end  % classdef
