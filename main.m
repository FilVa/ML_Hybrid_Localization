clear all
addpath(genpath('.\Functions'))

%% ----- Hybrid Localization ----- %
% This script produces 3 different trajectories to exemplify the use of
% main_algorithm. 
% Under section 'Input Variables' choose the desired parameters.


%% Input Variables
% --- Noise Parameters
range_deviation = 0.5; % standard deviation for ranges
heading_deviation = 1000; % concentration parameter for heading
angle_deviation=1000;% concentration parameter for bearings
speed_deviation =0.1;% standard deviation for speed
noise = [range_deviation,heading_deviation,angle_deviation,speed_deviation];

% --- Default Noise Parameters
range_dev_default =0.5;
heading_dev_default =1000;
angle_dev_default=1000;
speed_dev_default =0.1;
default_values = [range_dev_default,heading_dev_default,angle_dev_default,speed_dev_default];
 
% --- Parameter Estimation
%estimate parameters or use default values: 
%   0 : use default values
%   1 : estimate parameters
flag_param =0;
% use estimated parameters after time step : 
start_using_est_param = 40;

% --- Time Window
TIME_WINDOW = 5;

% --- Trajectory
% Options : 'laps' (2D),'lawnmower' (2D), 'helix' (3D)
trajectory = 'laps';

% --- Threshold for fista optimization
thresh = 10^(-3); 

% --- Number of time steps to estimate 
% if higher than number of positions available, it will be replaced by such
% value
n_iter=150;

% --- Number of nodes with bearing capability
% the first 'n_angle' nodes will be assumed to have angle capability
n_angle = 2;

% --- Maximum range at which nodes are able to obtain measurements. Nodes
% separated by superior distances will not have available measurements
% between them.
disk_radius = 1000;

%% Produce trajectory
% Computes positions and velocities for nodes and anchors.
% Outputs dimension, number of anchors, number of nodes and time step used
% when generating the trajectory.

[ anchors, vel_anchors, pos_nodes, vel_nodes, dim, n_anchors, n_nodes,time_step] = trajectory_generate(trajectory);


%% Computing ranges, bearings and velocities with noise

% Produce all the distance matrices (with noise) for all the instances, stacked horizontally
% first indices are from anchors, next for nodes
distance_matrices_stack  = create_dist_matrices(anchors,pos_nodes,dim,range_deviation,disk_radius);

% Produce all the angle matrices (with noise) for all the instances, stacked horizontally
% first indices are from anchors, next for nodes
% angles are in unit vector form
angle_matrices_stack  = create_angle_matrices(anchors,pos_nodes,dim,angle_deviation);

% Produce all the velocity matrices (with noise)
vel_nodes_n = create_vel_noisy(vel_nodes,dim,heading_deviation,speed_deviation);

%% Algorithm 

[est_nodes] = main_algorithm(thresh,distance_matrices_stack,angle_matrices_stack, anchors, vel_anchors, pos_nodes, vel_nodes, dim, n_anchors, n_nodes,vel_nodes_n,default_values,flag_param,TIME_WINDOW,time_step,n_iter,n_angle,start_using_est_param);

%% PLOTS

% plot true positions
plot_positions(anchors,pos_nodes,dim);

% plot estimates
plot_estimates_vs_true( anchors,pos_nodes,est_nodes,dim)