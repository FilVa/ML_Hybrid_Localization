function [ dist_vec ] = get_dist_stack( distances, edges , time_window )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

   n_points = size(distances,1);
   dist_vec = [];
   
   for t = 1:time_window
        for i = 1:size(edges,1);
            index = edges(i,:);
            d = distances(index(1),(t-1)*n_points+index(2));
            dist_vec = [dist_vec; d];
        end
    end


end

