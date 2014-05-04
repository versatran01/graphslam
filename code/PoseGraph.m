%%%
%> @file  PoseGraph.m
%> @brief A class for doing pose graph optimization
%%%
classdef PoseGraph < handle
    %POSEGRAPH A class for doing pose graph optimization
    
    properties (SetAccess = private)
        node  %> Pose nodes in graph
        edge  %> Edge in graph
        H     %> Information matrix
        b     %> Information vector
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
                vi = vertices(:,i_node);
                id = vi(1) + 1;
                pose = vi(2:4);
                obj.node(i_node) = PoseNode(id, pose);
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
            for i_edge = 1:size(edges,2)
                ei = edges(:,i_edge);
                id_from   = ei(1) + 1;
                id_to     = ei(2) + 1;
                mean      = ei(3:5);
                infm      = zeros(3,3);
                infm(1,1) = ei(6,:);
                infm(2,1) = ei(7,:);
                infm(1,2) = ei(7,:);
                infm(2,2) = ei(8,:);
                infm(3,3) = ei(9,:);
                infm(1,3) = ei(10,:);
                infm(3,1) = ei(10,:);
                infm(3,2) = ei(11,:);
                infm(2,3) = ei(11,:);
                obj.edge(i_edge) = PoseEdge(id_from, id_to, mean, infm);
            end
            fclose(efid);
        end
        
        %%%
        %> @brief Plot all nodes and edges
        %%%
        function plot(obj)
            obj.node.plot();
        end
        
        %%%
        %> @brief pose graph optimization
        %> @param n_iter Number of iteration to optimizae
        %> @param vis True to turn visualization on
        %%%
        function optimize(obj, n_iter, vis)
            if nargin < 3, vis = false; end
            if nargin < 2, n_iter = 1; end
            
            for i_iter = 1:n_iter
                fprintf('Pose Graph Optimization, Iteration %d.\n', i_iter);
                obj.iterate();
                fprintf('Iteration %d done.\n', i_iter);
                if vis
                    clf;
                    obj.plot();
                    drawnow
                end
            end
        end
        
        %%%
        %> @brief one iteration of linearization and solving
        %%%
        function iterate(obj)
            fprintf('Allocating Workspace.\n');
            % Create new H and b matrices each time
            obj.H = zeros(obj.n_node*3);   % 3n x 3n square matrix
            obj.b = zeros(obj.n_node*3,1); % 3n x 1  column vector
            
            fprintf('Linearizing.\n');
            obj.linearize();
            
            fprintf('Solving.\n');
            obj.solve();
        end
        
        %%%
        %> @brief Linearize error functions and formulate a linear system
        %%%
        function linearize(obj)
            for i_edge = 1:obj.n_edge
                ei = obj.edge(i_edge);
                % Get edge information
                i_node = ei.id_from;
                j_node = ei.id_to;
                T_z = v2t(ei.mean);
                omega = ei.infm;
                
                % Get node information
                v_i = obj.node(i_node).pose;
                v_j = obj.node(j_node).pose;
                i_ind = id2ind(i_node);
                j_ind = id2ind(j_node);
                
                T_i = v2t(v_i);
                T_j = v2t(v_j);
                R_i = T_i(1:2,1:2);
                R_z = T_z(1:2,1:2);
                
                si = sin(v_i(3));
                ci = cos(v_i(3));
                dR_i = [-si ci; -ci -si]';
                dt_ij = v_j(1:2) - v_i(1:2);
                
                % Caluclate jacobians
                A = [-R_z'*R_i' R_z'*dR_i'*dt_ij; 0 0 -1];
                B = [R_z'*R_i' [0;0]; 0 0 1];
                
                % Calculate error vector
                e = t2v(inv(T_z) * inv(T_i) * T_j);
                
                % Formulate blocks
                H_ii =  A' * omega * A;
                H_ij =  A' * omega * B;
                H_jj =  B' * omega * B;
                b_i  = -A' * omega * e;
                b_j  = -B' * omega * e;
                
                % Update H and b matrix
                obj.H(i_ind,i_ind) = obj.H(i_ind,i_ind) + H_ii;
                obj.H(i_ind,j_ind) = obj.H(i_ind,j_ind) + H_ij;
                obj.H(j_ind,i_ind) = obj.H(j_ind,i_ind) + H_ij';
                obj.H(j_ind,j_ind) = obj.H(j_ind,j_ind) + H_jj;
                obj.b(i_ind) = obj.b(i_ind) + b_i;
                obj.b(j_ind) = obj.b(j_ind) + b_j;
            end
        end
        
        %%%
        %> @brief solve the linear system and update all pose node
        %%%
        function solve(obj)
            fprintf('Pose: %d, Edge: %d\n', obj.n_node, obj.n_edge);
            % The system (H b) is obtained only from relative constraints.
            % H is not full rank.
            % We solve this by anchoring the position of the 1st vertex
            % This can be expressed by adding teh equation
            % dx(1:3,1) = 0
            % which is equivalent to the following
            obj.H(1:3,1:3) = obj.H(1:3,1:3) + eye(3);
            H_sparse = sparse(obj.H);
            dx = H_sparse \ obj.b;
            dpose = reshape(dx, 3, obj.n_node);
            
            % Update node with solution
            for i_node = 1:obj.n_node
                obj.node(i_node).pose = obj.node(i_node).pose ...
                    + dpose(:,i_node);
            end
        end
        
        % Get methods
        function n_node = get.n_node(obj)
            n_node = numel(obj.node);
        end
        
        function n_edge = get.n_edge(obj)
            n_edge = numel(obj.edge);
        end
        
        function pose = get.pose(obj)
            pose = [obj.node.pose];
        end
        
    end  % methods
    
end  % classdef


function ind = id2ind(id)
%ID2IND converts id to indices in H and b
ind = (3*(id-1)+1):(3*id);
end