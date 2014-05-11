classdef PoseEdge < handle
    %POSEEDGE A class for edge in a pose graph
    
    properties (SetAccess = private)
        id_from  % This is the viewing frame of this edge
        id_to    % This is the pose being observed from the viewing frame
        mean     % Predicted virtual measurement
        infm     % Information matrix of this edge
    end  % properties set private
    
    methods
        
        function obj = PoseEdge(id_from, id_to, mean, infm)
            % Consturctor of PoseEdge
            obj.id_from = id_from;
            obj.id_to = id_to;
            obj.mean = mean(:);
            obj.infm = infm;
        end
        
    end  % methods public
    
end  % classdef
