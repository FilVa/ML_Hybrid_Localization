function [ est_norm ,est_ang] = get_est_angle( alpha,est_nodes,M,dim )
%UNTITLED Summary of this function goes here
% get angle between 0 and 2pi, and norm
% [ est_ang ] =[norm angle]
%   Detailed explanation goes here
n_edges = size(M,1);
n_ang = size(M,1)/dim;
if(isempty(est_nodes))
    est_ang=[];
    est_norm=[];
    return
end
% Initialize alpha if emtpy
if(isempty(alpha))
    alpha=zeros(n_edges,1);
end
if(dim==2)
est_ang = zeros(n_ang,1);
elseif(dim==3)
  est_ang = zeros(n_ang,2);  
end
    est_norm = zeros(n_ang,1);

    % Difference between two positions
    est = M*est_nodes-alpha;


    for edge=1:n_ang
        % Get all dimensions for estimated difference
        ind_begin = (edge-1)*dim+1;
        ind_end = edge*dim;

        % Get all dimensions for estimated difference
        estimated = est(ind_begin:ind_end);
        if(dim==2)

        angle_est = wrapTo2Pi( atan2(estimated(2),estimated(1)));
        elseif(dim==3)
            angle =atan2(estimated(2),estimated(1));
            d_xy = norm(estimated(1:2));
                angle_z = atan2(estimated(3),d_xy);
                angle_est = [angle,angle_z];
 
        end
        norm_est = norm(estimated);

        est_norm(edge) = norm_est;
        est_ang(edge,:) = angle_est;

    end    
  
   
    


end
