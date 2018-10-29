function [ angle_matrices_stack ] =  create_angle_matrices(anchors,pos_nodes,dim,angle_deviation)
%create_angle_matrices Creates matrices of bearing measurements between
%all nodes and anchors. Each bearing with noise following von mises-fisher
%distribution, with concentration parameter = angle_deviation.
%   Input
%   - anchors : anchors positions
%   - pos_nodes : nodes positions
%   - dim : dimension
%   - angle_deviation : concentration parameter for noise
%   Output
%   - angle_matrices_stack : horizontal concatenation of matrices with
%   bearing measurements for each time step

% setup
n_iter = size(anchors,2); % number of measurements
angle_matrices_stack = [];
number_of_anchors = size(anchors,1)/dim;
unknowns = size(pos_nodes,1)/dim;
number_of_points = number_of_anchors + unknowns;

for ii = 1:n_iter
    points = [];    
    %Create matrix points
    %points = [anchor1_pos anchor2_pos ... node1_pos node2_pos ...]
    for jj = 1:number_of_anchors
        points = [points anchors((jj-1)*dim + 1:jj*dim, ii)];
    end
    
    for jj = 1:unknowns
        points = [points pos_nodes((jj-1)*dim + 1:jj*dim, ii)];
    end    
   
    ang_aux = zeros(number_of_points*dim, number_of_points);
    
    for kk = 1:number_of_points
        for hh = kk+1:number_of_points
            %Take angle between two points and add noise
            vec = points(:,hh) - points(:,kk); %nodes -anchors; and node_i-node_j (i>j)
            angle =atan2(vec(2),vec(1))+ vmrand(0,angle_deviation);            
            
            if(dim==2) % 2D case
                u = [cos(angle);sin(angle)];
            elseif(dim==3) % 3D case
                d_xy = norm(vec(1:2));
                angle_z = atan2(vec(3),d_xy)+vmrand(0,angle_deviation);
                u = [cos(angle)*cos(angle_z);sin(angle)*cos(angle_z);sin(angle_z)];
            end
            ang_aux(((hh-1)*dim+1:hh*dim),kk) = u;
            ang_aux(((kk-1)*dim+1:kk*dim),hh) = u;
        end
    end
     angle_matrices_stack  = [ angle_matrices_stack  ang_aux]; 
end

end

