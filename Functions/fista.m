function [solution] = fista(x_len,y_len,w_len,s_len,M,b,L,z_prev,dim,d,r,VT,time_window,thresh)
%fista Solves optimization problem with FISTA method
%   Input
%   - z_prev : previous solution z=(x,y,w,s)
%   - x_len : length of component x of vector z
%   - y_len : length of component y of vector z
%   - w_len : length of component w of vector z
%   - s_len : length of component s of vector z
%   - M and b : matrices defining cost function zMz-bz
%   - L : lipschitz constant
%   - dim : dimension
%   - d : vector of concatenated distances for node-node edges, along
%   time_window
%   - r : vector of concatenated distances for node-anchor edges, along
%   time_window
%   - VT : speed*time_step for all nodes, along time window
%   - time_window 
%   - thresh : stopping criteria for FIST iterations
%   Output
%   - solution : z=(x,y,w,s) after solving with fista

% Init
k=1; 
flag=1; % for stop
x_k_2 = z_prev;
x_k_1 = z_prev;



while(flag)
    y = x_k_1 + ((k-2)/(k+1))*(x_k_1-x_k_2);
    
    grad = M*y-b; %gradient
    
    point = y-(1/L)*grad; 
    
    x_k = projection(x_len,y_len,w_len,s_len,point,dim,d,r,VT,time_window); % projection   
    
    % check stop criteria
    norm(x_k-x_k_1);
    if(norm(x_k-x_k_1)<thresh)
        flag=0;
    end
    
    % update for next iteration
    x_k_2 = x_k_1;
    x_k_1 = x_k;
    k=k+1;
end

solution = x_k;

end


function [projected_point] = projection(x_size,y_size,w_size,s_size,point,dim,dist_nodes,dist_anchors,VT,time_window)

% Split vector into our components 
x = point(1:x_size);
y = point(x_size+1:x_size+y_size);
w = point(x_size+y_size+1:x_size+y_size+w_size);

[y_proj]=proj_vector(y,dim,dist_nodes); % project y component
[w_proj]=proj_vector(w,dim,dist_anchors); % project w component
x_proj = x; % x does not have projection

if(isempty(VT))%no velocity measurements
    projected_point = [x_proj;y_proj;w_proj];
else
    s = point(x_size+y_size+w_size+1:end);
    
    [s_proj]=proj_vector(s,dim,VT); % project s component
    
    projected_point = [x_proj;y_proj;w_proj;s_proj];
end
end

function [y_proj]=proj_vector(y,dim,d)
% proj_vector projects vector y, with constraints defined by vector d.
%   Input : 
%   - y : concatenation of vectors of dimension dim
%   - dim :dimension
%   - d : concatenation of distances corresponding to each vector of y
%   Output
%   - y_proj : projection of vector y

y_size = size(y,1);
y_proj = zeros(y_size,1);

for i=1:(y_size/dim) 
    
    p = y((i-1)*dim+1: i*dim); % for each vector of dimension dim
    dist = d((i-1)*dim+1); % correspondent distance in d
    center = zeros(dim,1); % center is zero
    proj = proj_ball_l2(p,center,dist); % projection of l2 norm ball
    y_proj((i-1)*dim+1: i*dim) = proj; % update projected vector
    
end

end

function [projected_point] = proj_ball_l2(point,center,dist)
%proj_ball_l2 projects point onto l2 norm ball of radius 'dist' and center 'center' 

if(norm(point-center)<=dist) % already fulfills condition, no need to project
    projected_point=point;
else
    projected_point =center+ dist*(point-center)/(norm(point-center));
end

end


