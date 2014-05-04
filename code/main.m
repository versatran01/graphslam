clear all
close all
clc

%% Specify files to load
vfile = '../data/killian-v.dat';
efile = '../data/killian-e.dat';

%% 
figure()
g = PoseGraph();
g.readGraph(vfile, efile);
% Do 5 iteration with visualization
g.optimize(5, true);
