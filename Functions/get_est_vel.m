function [ est_norm ,est_ang] = get_est_vel( est_nodes, time_step ,dim  )
%UNTITLED Summary of this function goes here
% get angle between 0 and 2pi, and norm
% [ est_ang ] =[norm angle]
%   Detailed explanation goes here
    
    n_ang = size(est_nodes,1)/dim;
  
    if(dim==2)
    est_ang = zeros(n_ang,1);
    elseif(dim==3)
        est_ang = zeros(n_ang,2);
    end
    est_norm = zeros(n_ang,1);
    
    k_1 = est_nodes(:,end);
    k_2 = est_nodes(:,end-1);
    k_3 = est_nodes(:,end-2);
    k_5 = est_nodes(:,end-4);
    k_6 = est_nodes(:,end-5);
    k_7 = est_nodes(:,end-6);
    
    diff = ((k_1-k_7)./32)+(4.*(k_2-k_6)./32)+(5*(k_3-k_5)./32);
    %diff = ((k_1-k_7)./60)-(9.*(k_2-k_6)./60)+(45*(k_3-k_5)./60);

    for edge=1:n_ang
        % Get all dimensions for estimated difference
        ind_begin = (edge-1)*dim+1;
        ind_end = edge*dim;

        % Get all dimensions for estimated difference
        estimated = diff(ind_begin:ind_end);
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

