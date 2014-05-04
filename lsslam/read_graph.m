%   This source code is part of the graph optimization package
%   deveoped for the lectures of robotics2 at the University of Freiburg.
%  
%     Copyright (c) 2007 Giorgio Grisetti, Gian Diego Tipaldi
%  
%   It is licences under the Common Creative License,
%   Attribution-NonCommercial-ShareAlike 3.0
%  
%   You are free:
%     - to Share - to copy, distribute and transmit the work
%     - to Remix - to adapt the work
%  
%   Under the following conditions:
%  
%     - Attribution. You must attribute the work in the manner specified
%       by the author or licensor (but not in any way that suggests that
%       they endorse you or your use of the work).
%    
%     - Noncommercial. You may not use this work for commercial purposes.
%    
%     - Share Alike. If you alter, transform, or build upon this work,
%       you may distribute the resulting work only under the same or
%       similar license to this one.
%  
%   Any of the above conditions can be waived if you get permission
%   from the copyright holder.  Nothing in this license impairs or
%   restricts the author's moral rights.
%  
%   This software is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied 
%   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
%   PURPOSE.  

%loads a graph file in a list
%vfile: the input vertex file
%	every line is made as follows
%	VERTEX2 id pose.x pose.y pose.theta
%efile: the input edge file 
%	every line is made as follows
%	EDGE2 idFrom idTo mean.x mean.y mean.theta inf.xx inf.xy inf.yy inf.xt inf.yt inf.tt
%vmeans: matrix containing the column vectors of the poses of the vertices
%	 the vertices are odrered such that vmeans[i] corresponds to the ith id
%eids:	 matrix containing the column vectors [idFrom, idTo]' of the ids of the vertices
%	 eids[k] corresponds to emeans[k] and einfs[k].
%emeans: matrix containing the column vectors of the poses of the edges
%einfs:  3d matrix containing the information matrices of the edges
%	 einfs(:,:,k) refers to the information matrix of the k-th edge.
%IMPORTANT: the ids in the file start with 0, the ids in matlab/octave start with 1

function [vmeans, eids, emeans, einfs]=read_graph(vfile, efile)
	ef = fopen(efile);
	vf = fopen(vfile);
	vertices = fscanf(vf, 'VERTEX2 %d %f %f %f\n', [4,Inf]);
	edges = fscanf(ef,'EDGE2 %d %d %f %f %f %f %f %f %f %f %f \n',[11,Inf]);
    
    % Xi
	vmeans = vertices(2:4, vertices(1,:)+1);
    
    % [i, j]
	eids = edges(1:2,:) + 1;
    
    % Zij
	emeans = edges(3:5,:);
    
    % Information matrix
	einfs(1,1,:) = edges(6,:);
	einfs(2,1,:) = edges(7,:);
	einfs(1,2,:) = edges(7,:);
	einfs(2,2,:) = edges(8,:);
	einfs(3,3,:) = edges(9,:);
	einfs(1,3,:) = edges(10,:);
	einfs(3,1,:) = edges(10,:);
	einfs(3,2,:) = edges(11,:);
	einfs(2,3,:) = edges(11,:);
end

