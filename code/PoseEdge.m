%%%
%> @file PoseNode.m
%> @brief A class for edge in a pose graph
%%%
classdef PoseEdge < handle
    %POSEEDGE A class for edge in a pose graph
    
    properties (SetAccess = private)
        id_from
        id_to
        mean
        infm     %> information matrix of this edge
    end
    
    methods
        function obj = PoseEdge(id_from, id_to, mean, infm)
            obj.id_from = id_from;
            obj.id_to = id_to;
            obj.mean = mean;
            obj.infm = infm;
        end
    end
    
end

