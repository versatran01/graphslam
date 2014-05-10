clear all
close all
clc

%% Specify files to load
vfile = '../data/killian-v.dat';
efile = '../data/killian-e.dat';

%% 
figure()
pg = PoseGraph();
pg.readGraph(vfile, efile);
% Do 5 iteration with visualization
pg.optimize(5, true);
