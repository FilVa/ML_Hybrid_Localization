function [ M,b ] = quadratic_form(n_edges_nodes,n_edges_anchors,sz_nd,sz_vel,sz_anchor,sz_nodes,sigma_est_nodes,sigma_est_anchors,sigma_est_vel,A_time,E,alpha,N,kappa_est_nodes,kappa_est_anchors,kappa_vel,vec_u,vec_v,vec_sR)
%quadratic_form Reformulates the problem into a quadratic expression 0.5zMz
%- b


% Parameters
if(n_edges_nodes>0)
    S_N = sigma_est_nodes;
else
    S_N =[];
end
if(n_edges_anchors>0)
    S_A = sigma_est_anchors;
else
    S_A=[];
end
S_V = sigma_est_vel;

% Multiply each matrix by respective diagonal parameters
A_til = diag(S_N)*A_time;
E_time_til = diag(S_A)*E;
alpha_til = diag(S_A)*alpha;
N_til = diag(S_V)*N;


if(sz_vel==0)
    M1 = [A_til,-diag(S_N)*eye(sz_nd), zeros(sz_nd,sz_anchor) ];
    M2 = [E_time_til, zeros(sz_anchor,sz_nd), -diag(S_A)*eye(sz_anchor)];
    M3=[];
else
    M1 = [A_til,-diag(S_N)*eye(sz_nd), zeros(sz_nd,sz_anchor),zeros(sz_nd,sz_vel) ];
    M2 = [E_time_til, zeros(sz_anchor,sz_nd), -diag(S_A)*eye(sz_anchor),zeros(sz_anchor,sz_vel)];
    M3 = [N_til,zeros(sz_vel,sz_nd),zeros(sz_vel,sz_anchor),-diag(S_V)*eye(sz_vel) ];
end


b1 = M2'*alpha_til;

if(isempty(vec_u))
    u_til=[];
else
    u_til = kappa_est_nodes.*vec_u;
end
if(isempty(vec_v))
    q_til = [];
else
    q_til = kappa_est_anchors.*vec_v;
end

if(isempty(M3))
    M =M1'*M1 + M2'*M2 ;
    b2= [zeros(sz_nodes,1);u_til;q_til];
else
    M =M1'*M1 + M2'*M2 + M3'*M3 ;
    vel_til = (kappa_vel.*vec_sR);
    b2= [zeros(sz_nodes,1);u_til;q_til;vel_til];
end
b=b1+b2;

end

