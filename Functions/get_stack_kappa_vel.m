function [new_stack] = get_stack_kappa_vel(old_stack,est_nodes,M,dim,current_vec,alpha)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
if(isempty(alpha))
    alpha=zeros(size(M,1),1);
end

est = M*est_nodes-alpha;

new_stack = zeros(size(old_stack));

for edge=1:length(old_stack)/dim
    ind_begin = (edge-1)*dim+1;
    ind_end = edge*dim;
    n = norm(est(ind_begin:ind_end));
    
    est_norm = est(ind_begin:ind_end)./n;
    measured = current_vec(ind_begin:ind_end);
    
    
    dif = measured - est_norm;
    new_term =dif;
    new_stack(ind_begin:ind_end) = old_stack(ind_begin:ind_end) + new_term;
end

end

