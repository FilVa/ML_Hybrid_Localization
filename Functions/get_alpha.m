function [ alpha ] = get_alpha(anchors,australian_edges,dimension)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here

    alpha = []; %stack of anchor positions
    
    %initialization of alpha, according to australian_edges   
    for ii = 1:size(anchors,2)
        for jj = 1:size(australian_edges)
            indexa = australian_edges(jj,1);
            alpha = [alpha; anchors(dimension*(indexa-1) + 1:dimension*indexa , ii)];
        
        end
    end
end

