%%%
%> @file  PoseGraph.m
%> @brief A class for doing pose graph optimization
%%%
classdef PoseGraph < handle
    %POSEGRAPH A class for doing pose graph optimization
    
    properties (SetAccess = private)
        node  %> Pose nodes in graph
        edge  %> Edge in graph
    end
    
    properties (Dependent = true)
        n_node  %> Number of nodes in graph
        n_edge  %> Number of edges in graph
        pose    %> Poses of all nodes
    end
    
    methods
        %%%
        %> @brief Class constructor
        %> Instantiates an object of GraphSlam
        %>
        %> @return instance of the GraphSlam class
        %%%
        function obj = PoseGraph()
            obj.node = PoseNode.empty;
            obj.edge = PoseEdge.empty;
        end
        
        function readGraph(obj, vfile, efile)
            % Try opening vertex file
            vfid = fopen(vfile);
            if (vfid < 0)
                fprintf('Fail to open %s\n.', vfile);
                return
            end
            vertices = fscanf(vfid, 'VERTEX2 %d %f %f %f\n', [4 Inf]);
            for i_node = 1:size(vertices,2)
                vertex = vertices(:,i_node);
                obj.node(i_node) = PoseNode(vertex(1), vertex(2:4));
            end
            fclose(vfid);
            % Try opening edge file
            efid = fopen(efile);
            if (efid < 0)
                fprintf('Fail to open %s\n.', vfile);
                return
            end
            edges = fscanf(efid,...
                'EDGE2 %d %d %f %f %f %f %f %f %f %f %f \n',[11 Inf]);
            
        end
        
        function n_node = get.n_node(obj)
            n_node = numel(obj.node);
        end
        
        function n_edge = get.n_edge(obj)
            n_edge = numel(obj.edge);
        end
        
        function pose = get.pose(obj)
            pose = [obj.node.pose];
        end
    end
end

