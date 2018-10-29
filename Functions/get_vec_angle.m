function [  ang_u] = get_vec_angle(angles,edge_array_nodes,dim,time_window,n_points,n_with_angle)
%UNTITLED18 Summary of this function goes here
%   Detailed explanation goes here

    ang_u =[];
    for t = 1:time_window
        for i = 1:size(edge_array_nodes,1);
            index = edge_array_nodes(i,:);
            if(ismember(index(1),n_with_angle)   || ismember(index(2),n_with_angle)  )
                ang_vec = angles((index(1)-1)*dim+1:index(1)*dim,(t-1)*n_points+index(2));
            else
                ang_vec=zeros(dim,1);
            end
            
            ang_u = [ang_u; ang_vec];
        end
    end
 
end

