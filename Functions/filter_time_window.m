function  [window_matrices] = filter_time_window(matrices_stack,ii,time_window,sz)
%UNTITLED gets the a sub matrix of 'matrices_stack' with length time_window
%up until ii, where 'sz' is the size of each window slice.
%   Detailed explanation goes here
    window_matrices = matrices_stack(:,(sz*(ii-time_window )) + 1:sz*(ii));
end

