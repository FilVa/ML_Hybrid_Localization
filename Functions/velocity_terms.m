function [v,V,vec_vV] = velocity_terms(vel_matrix,dim,time_window,time_step,i)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

if(i==1)
    v=[];
    V=[];
    vec_vV=[];
else
    vel_stack = get_window_stack(vel_matrix, i,time_window,1);

    [v,V] = split_unit_norm(vel_stack,dim);

    % vec_vV is the complete vector to insert in optimization problem
    vec_vV = v./(V.*time_step);
    vec_vV(isnan(vec_vV))=0; % in case norm is zero, unit vector is nan
  
end

end

