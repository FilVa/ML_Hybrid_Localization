function [ param_est,stack] = parameters_init(type, fix_value, number_edges,dim,time_window)
%parameters_init Extends 'fix_value' into array with desired dimensions and
%initializes stack with zeros
%   Input
%   - type : 'sigma' (std of gaussian distribution) or 'kappa' (concentration parameter of vMF)
%   - fix_value : value to init parameters
%   - number_edges : number of edges
%   - dim : dimension
%   - time_window
%   Output
%   - param_est : initialization of parameter vector with 'fix_value'
%   - stack : initialization of stack for parameter estimation with zeros

    param_est = repmat(fix_value,number_edges*dim*time_window,1);
    if(strcmp(type,'sigma'))
        stack=zeros(number_edges,1);
    elseif(strcmp(type,'kappa'))
        stack=zeros(number_edges*dim,1);
    end
 end

