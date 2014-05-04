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


%ls-slam.m
%this file is released under the creative common license

%solves a graph-based slam problem via least squares
%vmeans: matrix containing the column vectors of the poses of the vertices
%	 the vertices are odrered such that vmeans[i] corresponds to the ith id
%eids:	 matrix containing the column vectors [idFrom, idTo]' of the ids of the vertices
%	 eids[k] corresponds to emeans[k] and einfs[k].
%emeans: matrix containing the column vectors of the poses of the edges
%einfs:  3d matrix containing the information matrices of the edges
%	 einfs(:,:,k) refers to the information matrix of the k-th edge.
%n:	 number of iterations
%newmeans: matrix containing the column vectors of the updated vertices positions

function newmeans = ls_slam(vmeans, eids, emeans, einfs, n)

for i = 1:n
    vmeans = linearize_and_solve(vmeans, eids, emeans, einfs);
end

newmeans = vmeans;

end


%computes the taylor expansion of the error function of the k_th edge
%vmeans: vertices positions
%eids:   edge ids
%emeans: edge means
%k:	 edge number
%e:	 e_k(x)
%A:	 d e_k(x) / d(x_i)
%B:	 d e_k(x) / d(x_j)
function [e, A, B] = linear_factors(vmeans, eids, emeans, k)
%extract the ids of the vertices connected by the kth edge
id_i = eids(1,k);
id_j = eids(2,k);
%extract the poses of the vertices and the mean of the edge
v_i = vmeans(:,id_i);
v_j = vmeans(:,id_j);
z_ij = emeans(:,k);

%compute the homoeneous transforms of the previous solutions
zt_ij = v2t(z_ij);
vt_i = v2t(v_i);
vt_j = v2t(v_j);

%compute the displacement between x_i and x_j
f_ij=(inv(vt_i) * vt_j);

%this below is too long to explain, to understand it derive it by hand
theta_i = v_i(3);
ti = v_i(1:2,1);
tj = v_j(1:2,1);
dt_ij = tj-ti;

si = sin(theta_i);
ci = cos(theta_i);

A= [-ci, -si, [-si, ci]*dt_ij; si, -ci, [-ci, -si]*dt_ij; 0, 0, -1 ];
B =[  ci, si, 0           ; -si, ci, 0            ; 0, 0, 1 ];

ztinv = inv(zt_ij);
e = t2v(ztinv * f_ij);
ztinv(1:2,3) = 0;
A = ztinv*A;
B = ztinv*B;
end


%linearizes and solves one time the ls-slam problem specified by the input
%vmeans:   vertices positions at the linearization point
%eids:     edge ids
%emeans:   edge means
%einfs:    edge information matrices
%newmeans: new solution computed from the initial guess in vmeans
function newmeans = linearize_and_solve(vmeans, eids, emeans, einfs)
disp('allocating workspace...');
% H and b are respectively the system matrix and the system vector
H = zeros(size(vmeans,2)*3);
b = zeros(size(vmeans,2)*3,1);

disp('linearizing');
% this loop constructs the global system by accumulating in H and b the contributions
% of all edges (see lecture)
for k = 1:size(eids,2),
    id_i = eids(1,k);
    id_j = eids(2,k);
    [e, A, B] = linear_factors(vmeans, eids, emeans,  k);
    omega = einfs(:,:,k);
    %compute the blocks of H^k
    b_i = -A' * omega * e;
    b_j = -B' * omega * e;
    H_ii=  A' * omega * A;
    H_ij=  A' * omega * B;
    H_jj=  B' * omega * B;
    
    %accumulate the blocks in H and b
    H((id_i-1)*3+1:id_i*3,(id_i-1)*3+1:id_i*3) = ...
        H((id_i-1)*3+1:id_i*3,(id_i-1)*3+1:id_i*3)+ H_ii;
    H((id_j-1)*3+1:id_j*3,(id_j-1)*3+1:id_j*3) = ...
        H((id_j-1)*3+1:id_j*3,(id_j-1)*3+1:id_j*3) + H_jj;
    H((id_i-1)*3+1:id_i*3,(id_j-1)*3+1:id_j*3) = ...
        H((id_i-1)*3+1:id_i*3,(id_j-1)*3+1:id_j*3) + H_ij;
    H((id_j-1)*3+1:id_j*3,(id_i-1)*3+1:id_i*3) = ...
        H((id_j-1)*3+1:id_j*3,(id_i-1)*3+1:id_i*3) + H_ij';
    b((id_i-1)*3+1:id_i*3,1) = ...
        b((id_i-1)*3+1:id_i*3,1) + b_i;
    b((id_j-1)*3+1:id_j*3,1) = ...
        b((id_j-1)*3+1:id_j*3,1) + b_j;
    
    %NOTE on Matlab compatibility: note that we use the += operator which is octave specific
    %using H=H+.... results in a tremendous overhead since the matrix would be entirely copied every time
    %and the matrix is huge
end;
disp('Done');
%note that the system (H b) is obtained only from
%relative constraints. H is not full rank.
%we solve the problem by anchoring the position of
%the the first vertex.
%this can be expressed by adding the equation
%  deltax(1:3,1)=0;
%which is equivalent to the following
H(1:3,1:3) = H(1:3,1:3) + eye(3);

SH = sparse(H);
disp('System size: '),disp(size(H));
disp('solving (may take some time) ...');
deltax = SH\b;
disp('Done! ');

%split the increments in nice 3x1 vectors and sum them up to the original matrix
newmeans = vmeans + reshape(deltax, 3, size(vmeans,2));

disp('Normalizing the angles');
%normalize the angles between -PI and PI
for i = 1:size(newmeans,2)
    s = sin(newmeans(3,i));
    c = cos(newmeans(3,i));
    newmeans(3,i) = atan2(s,c);
end
disp('Done');
end
