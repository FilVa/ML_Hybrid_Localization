function [L]=lipschitz(distances,n_anchors,n_points,time_window,sigma_est_nodes,sigma_est_anchors,sigma_est_vel);
%lipschitz Computes Lipschitz constant for current time step
%   Input
%   - distances : matrix with distances between nodes and anchors
%   - n_anchors : number of anchors
%   - time_window
%   - sigma_est_nodes : vector of standard deviation for each edge
%   node-node
%   - sigma_est_anchors : vector of standard deviation for each edge
%   node-anchor
%   - sigma_est_vel : vector of standard deviation for each edge
%   velocity
%   Output
%   - L : Lipschitz constant


% where do we have edges
check_matrix = distances(:, 1:n_points) > 0;
% node degrees
nd = sum(check_matrix(n_anchors+1:end,n_anchors+1:end),2);
% maximum node degree
dmax = max(nd);
% maximum node-anchor edges
maxAi = max(sum(check_matrix(1:n_anchors,n_anchors+1:end),1));
% velocity node degree, depends on time window
if(time_window==1)
    dmax1=0;
elseif(time_window==2)
    dmax1=1;
else
    dmax1=2;
end
% maximum sigmas
sum_sigma = (max(sigma_est_nodes)^2)+(max(sigma_est_anchors)^2)+(max(sigma_est_vel)^2);

L = (2*dmax)*(max(sigma_est_nodes)^2)+maxAi*(max(sigma_est_anchors)^2)+2*dmax1*(max(sigma_est_vel)^2)+sum_sigma;


end

