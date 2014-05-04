disp('least squares slam example\n');
disp('loading the graph file from the vertices and edge description\n');

[vmeans, eids, emeans, einfs] = read_graph('data/killian-v.dat', 'data/killian-e.dat');

%% this plots the input trajectory
plot (vmeans(1,:),vmeans(2,:))
pause(1);

%% get the vertices after 1 iteration of lse
v = ls_slam(vmeans, eids, emeans,  einfs, 1);

%% this plots result
plot (v(1,:),v(2,:))
pause(1);

%% do another iteration (starting from the previous solution)
v=ls_slam(v, eids, emeans,  einfs, 1);
plot (v(1,:),v(2,:))
pause(1);

%% ok, now we finalize the process with 4 iterations
v = ls_slam(v, eids, emeans,  einfs, 4);
plot (v(1,:),v(2,:))
pause(1);

%%
disp('example with small rotational information values (increased effect of non-linearities, takes longer to converge)\n');
disp('loading the graph file from the vertices and edge description\n');
% this loads the means of the vertices, the means of the edges, the edges ids and the edges information
% from the dat files
[vmeans, eids, emeans,  einfs] = read_graph('data/killian-v.dat', 'data/killian-small-rot-inf-e.dat');

% this plots the input trajectory
plot (vmeans(1,:),vmeans(2,:));
pause(1);

% get the vertices after 1 iteration of lse
v = ls_slam(vmeans, eids, emeans,  einfs, 1);

% this plots result
plot (v(1,:),v(2,:))
pause(1);

% do another iteration (starting from the previous solution)
v = ls_slam(v, eids, emeans,  einfs, 1);
plot (v(1,:),v(2,:))
pause(1);

% ok, now we finalize the process with 4 iterations
v = ls_slam(v, eids, emeans,  einfs, 4);
plot (v(1,:),v(2,:))
pause(1);
