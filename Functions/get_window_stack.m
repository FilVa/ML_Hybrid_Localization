function [ stack ] = get_window_stack(M, i,time_window,sz )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    if(time_window<=0 || isempty(M))
        stack = [];
    else
        
        M_filter = filter_time_window(M,i,time_window,sz);
        stack = reshape(M_filter,[size(M_filter,2)*size(M_filter,1),1]);
    end
end

