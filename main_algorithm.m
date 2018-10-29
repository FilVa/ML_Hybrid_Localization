function [est_nodes] = main_algorithm(thresh,distance_matrices_stack,angle_matrices_stack, anchors, vel_anchors, pos_nodes, vel_nodes, dim, n_anchors, n_nodes,vel_nodes_n,default_values,flag_param,TIME_WINDOW,time_step,n_iter_user,n_angle,start_using_est_param)


%% Setup

% --- Default Noise Parameters to use in first iterations and 
% for the remaining if parameter estimation is not performed (flag_param==0)
sigma =default_values(1);
kappa_vel =default_values(2);
kappa=default_values(3);
sigma_vel =default_values(4);

window_vel =7; % required because of shift in velocity estimate
start_param_est = window_vel+2; % iteration to start estimate parameters. at least window_vel + 2

% Create the indices of nodes with angle capability
if(n_angle>n_nodes)
    n_angle=n_nodes;
end
idx_nodes_with_angle = [n_anchors+1:1:n_anchors+n_angle];

n_points = n_nodes + n_anchors;

%% Iterations
if(n_iter_user >size(pos_nodes,2))
    n_iter = size(pos_nodes,2);
else
    n_iter= n_iter_user;
end

%% Solve each measurement

est_nodes = [];
est_nodes_new=[];

for i = 1:n_iter
    
    %% For the first iterations
    if i < TIME_WINDOW
        time_window = i;
        time_window_param = i+1;
    elseif i == TIME_WINDOW
        time_window = i;
        time_window_param = TIME_WINDOW;
    else
        time_window = TIME_WINDOW;
        time_window_param = TIME_WINDOW;
    end
    
    %% Horizontal stack of time window measurements (first columns older, last columns most recent)
    distances = filter_time_window(distance_matrices_stack,i,time_window,n_points);
    angles = filter_time_window(angle_matrices_stack,i,time_window,n_points);
      
    
    %% ------- Edges and corespondent matrices ------- %
    
    % Edges between node-node, node-anchor, number of edges
    % node-node,number of edges node-anchor
    [ edge_array_nodes,edge_array_anchors,n_edges_nodes,n_edges_anchors] = get_edges( distances,n_anchors,n_points );
    
    % Arc-incidence matrix with respect to distance measurements, for
    % node-node edges.
    % A : extended over dimension; A_time : A extended over time window
    [A,A_time] = get_matrix_A(edge_array_nodes,n_anchors,n_nodes,dim,time_window);
    
    % Selector matrix to match nodes with correspondent measurements with
    % respect to anchors.
    % E : extended over dimension; E_time : E extended over time window
    [E,E_time] = get_matrix_E(distances,n_anchors,n_nodes,dim,time_window);
    
    %Matrix N : to create x(t)-x(t-1)
    N = get_matrix_N(n_nodes,dim,time_window);
    
    %% -------- Vectors for measurements ------- %
    
    % Anchors position stack according to australian edges: positions column vector along time_window
    alpha = get_alpha(anchors(:,(i-time_window+1):i),edge_array_anchors,dim);
    
    % Distances stacks for nodes and anchors
    dist_vec_nodes = get_dist_stack( distances, edge_array_nodes, time_window);
    dist_nodes_dim = repelem(dist_vec_nodes,dim,1); %repeat distances to match dimension vectors
    dist_vec_anchors = get_dist_stack( distances, edge_array_anchors, time_window);
    dist_anchors_dim = repelem(dist_vec_anchors,dim,1);%repeat distances to match dimension vectors
    r = dist_anchors_dim;
    d = dist_nodes_dim;
     
    % Angles stack according to australian edges
    % Nodes - nodes
    ang_u = get_vec_angle(angles,edge_array_nodes,dim,time_window,n_points,idx_nodes_with_angle); % ang_u unit vectors
    vec_u = ang_u./dist_nodes_dim; %vec_u = ang_u divided by respective distance
    % Nodes - anchors
    ang_v = get_vec_angle(angles,edge_array_anchors,dim,time_window,n_points,idx_nodes_with_angle);%ang_v unit vectors
    vec_v = ang_v./dist_anchors_dim; %vec_v = ang_u divided by respective distance
    
    % Velocity stacks
    % Velocity stack vectors (1st iteration empty vectors)
    [s_vec,R,vec_sR] = velocity_terms(vel_nodes_n,dim,time_window,time_step,i);
    VT = R.*time_step;
    
    
    
    
    %% ------- Initialize parameter estimates with default values until estimate begins ------- %
    if(i<start_param_est) % always use default before parameters estimation begins
        [sigma_est_nodes,stack_std_nodes] = parameters_init('sigma',1/(sigma),n_edges_nodes,dim,time_window);
        [sigma_est_anchors,stack_std_anchors]=parameters_init('sigma',1/(sigma),n_edges_anchors,dim,time_window);
        [kappa_est_nodes,stack_kappa_nodes ]=parameters_init('kappa',kappa,n_edges_nodes,dim,time_window);
        [kappa_est_anchors,stack_kappa_anchors] =parameters_init('kappa',kappa,n_edges_anchors,dim,time_window);
        [kappa_est_vel,stack_kappa_vel] =parameters_init('kappa',kappa_vel,n_nodes,dim,time_window);
        [sigma_est_vel,stack_sigma_vel] =parameters_init('sigma',1/sigma_vel,n_nodes,dim,time_window);
        
     elseif(i<start_using_est_param) % use default values until 'start_using_est_param'
        [sigma_est_nodes,stub] = parameters_init('sigma',1/(sigma),n_edges_nodes,dim,time_window);
        [sigma_est_anchors,stub]=parameters_init('sigma',1/(sigma),n_edges_anchors,dim,time_window);
        [kappa_est_nodes,stub ]=parameters_init('kappa',kappa,n_edges_nodes,dim,time_window);
        [kappa_est_anchors,stub] =parameters_init('kappa',kappa,n_edges_anchors,dim,time_window);
        [kappa_est_vel,stub] =parameters_init('kappa',kappa_vel,n_nodes,dim,time_window);
        [sigma_est_vel,stub] =parameters_init('sigma',1/sigma_vel,n_nodes,dim,time_window);
    end
    
    
    %% ------- Matrices for Quadratic Form ----------%
    % Sizes
    sz_nd = n_edges_nodes*dim*time_window;
    sz_vel = n_nodes*dim*time_window;
    sz_anchor = n_edges_anchors*dim*time_window;
    sz_nodes = n_nodes*dim*time_window;
    
    if(isempty(vec_sR))
        vec_sR = zeros(sz_vel,1);
        VT =zeros(sz_vel,1);
    end
    
    % Obtain matrices M and b
    [M,b] =quadratic_form(n_edges_nodes,n_edges_anchors, sz_nd,sz_vel,sz_anchor,sz_nodes,sigma_est_nodes,sigma_est_anchors,sigma_est_vel,A_time,E_time,alpha,N,kappa_est_nodes,kappa_est_anchors,kappa_est_vel,vec_u,vec_v,vec_sR);
    
    
    
    %% ---- Lipschitz Constant ---- %
        
    L=lipschitz(distances,n_anchors,n_points,time_window,sigma_est_nodes,sigma_est_anchors,sigma_est_vel);
    
    %% ------- Optimization ------- %   
    x_len = dim*n_nodes*time_window;
    y_len = time_window*n_edges_nodes*dim;
    w_len = time_window*n_edges_anchors*dim;
    s_len =n_nodes*dim*(time_window);
    
    % Initialization of vector z (any values can be used)
    if(i<=time_window)
        z_prev = zeros(x_len+y_len+w_len+s_len,1);
    end
    
    % FISTA method
    [solution] = fista(x_len,y_len,w_len,s_len,M,b,L,z_prev,dim,d,r,VT,time_window,thresh);
    
    z_prev = solution; % update vector z to use in next iteration
    est_nodes_new = solution(n_nodes*dim*(time_window-1)+1:n_nodes*dim*time_window); % take only x component
    est_nodes = [est_nodes est_nodes_new]; % store position estimates
    

    
    %% ------- Current measured values ------- %
    % previous vector contain measurements for entire time window. Take
    % only the measurements of this time step. Necessary for parameter
    % estimation.
    
    % Anchors Position
    current_alpha = alpha(end-dim*n_edges_anchors+1:end);
    % Distances
    current_dist_nodes = dist_vec_nodes(end-n_edges_nodes+1:end);
    current_dist_anchors = dist_vec_anchors(end-n_edges_anchors+1:end);
    % Angles
    current_ang_u = ang_u(end-dim*n_edges_nodes+1:end); % Unit vector node-node
    [not_used, current_ang_node] = get_est_angle( [],current_ang_u,eye(length(current_ang_u)),dim); % in angle form (not unit vector)
    current_ang_v = ang_v(end-dim*n_edges_anchors+1:end); % Unit vector node-anchors
    [not_used, current_ang_anchors] = get_est_angle( [],current_ang_v,eye(length(current_ang_v)),dim);% in angle form (not unit vector)
    % Velocities
    [current_vel, not_used] = split_unit_norm(vel_nodes_n(:,i),dim);

    %% ------- Current estimated values ------- %
    if(i>window_vel)
        
        [delay_vel, not_used] = split_unit_norm(vel_nodes_n(:,i-3),dim);
        [current_dist_vel, current_ang_vel] = get_est_angle( [],delay_vel,eye(length(current_vel)),dim); % angle form
        
        %% ------- Estimated values ------- %
        % from obtained position estimate, compute estimated measurements.
        % necessary for parameter estimation
        [ est_dist_nodes, est_ang_nodes ] = get_est_angle( [],est_nodes_new,A,dim);
        [ est_dist_anchors, est_ang_anchors] = get_est_angle( current_alpha,est_nodes_new,E,dim  );
        [ est_norm_vel, est_ang_vel ] = get_est_vel( est_nodes, time_step ,dim  );
         
    end
    
    %% ------- Parameter estimation ------- %
    
    if(i>=(start_param_est-1))
        n_val = i - start_param_est + 2; % number of time steps with estimate
        
        % Sigma distances estimate
        [sigma_est_nodes,stack_std_nodes] = parameters_est(n_val,'sigma',flag_param,1/(sigma),dim,time_window_param,n_edges_nodes,...
            stack_std_nodes,current_dist_nodes,est_dist_nodes);
        [sigma_est_anchors,stack_std_anchors] = parameters_est(n_val,'sigma',flag_param,1/(sigma),dim,time_window_param,n_edges_anchors,...
            stack_std_anchors,current_dist_anchors,est_dist_anchors);
        
        % Kappa angles estimate
        [kappa_est_nodes,stack_kappa_nodes ]= parameters_est(n_val,'kappa',flag_param,kappa,dim,time_window_param,n_edges_nodes,...
            stack_kappa_nodes,current_ang_node,est_ang_nodes);
        [kappa_est_anchors,stack_kappa_anchors] =  parameters_est(n_val,'kappa',flag_param,kappa,dim,time_window_param,n_edges_anchors,...
            stack_kappa_anchors,current_ang_anchors,est_ang_anchors);
        
        % Kappa velocity estimate
        [kappa_est_vel,stack_kappa_vel] = parameters_est(n_val,'kappa',flag_param,kappa_vel,dim,time_window_param,n_nodes,...
            stack_kappa_vel,current_ang_vel,est_ang_vel);
        [sigma_est_vel,stack_sigma_vel] = parameters_est(n_val,'sigma',flag_param,1/sigma_vel,dim,time_window_param,n_nodes,...
            stack_sigma_vel,current_dist_vel,est_norm_vel);
    end
    
end

