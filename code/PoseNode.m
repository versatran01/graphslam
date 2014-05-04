%%%
%> @file PoseNode.m
%> @brief A class for node in a pose graph
%%%
classdef PoseNode < handle
    %POSENODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id    %> Id of this pose node
        pose  %> Pose of this pose node
    end
    
    properties (Dependent =  true)
        x
        y
        yaw
        rt     %> Transformation local to global
    end
    
    methods
        %%%
        %> @brief Class constructor
        %> Instantiates an object of PoseNode
        %>
        %> @param pose a robot pose in 2d [x; y; yaw]
        %> @return instance of the PoseNode class
        %%%
        function obj = PoseNode(id, pose)
            obj.id   = id;
            obj.pose = pose(:);
        end
        
        % Get methods
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
        
    end  % methods
    
end  % classdef
