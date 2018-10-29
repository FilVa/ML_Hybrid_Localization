function [ distance_matrices_stack ] = create_dist_matrices(anchors,pos_nodes,dim,range_deviation,disk_radius)
%create_dist_matrices Creates matrices of distance measurements between
%all nodes and anchors. Each measurement with noise following Gaussian
%distribution, with standard deviation = range_deviation.
%   Input
%   - anchors : anchors positions
%   - pos_nodes : nodes positions
%   - dim : dimension
%   - range_deviation : standard deviation for noise
%   - disk_radius : maximum range for nodes to obtain measurements.
%   Distances higher than this value will be set to zero.
%   Output
%   - distance_matrices_stack : horizontal concatenation of matrices with
%   range measurements for each time step


n_iter = size(anchors,2); % number of measurements
distance_matrices_stack = [];
number_of_anchors = size(anchors,1)/dim;
unknowns = size(pos_nodes,1)/dim;
number_of_points = number_of_anchors + unknowns;

for ii = 1:n_iter
    points = [];
    
    %Create matrix points
    %points = [anchor1_pos anchor2_pos node1_pos node2_pos]
    for jj = 1:number_of_anchors
        points = [points anchors((jj-1)*dim + 1:jj*dim, ii)];
    end
    
    for jj = 1:unknowns
        points = [points pos_nodes((jj-1)*dim + 1:jj*dim, ii)];
    end
    
    %Create d_aux at each iteration
    %d_aux =[0 d12 d13 d14
    %        d21 0 d23 d24
    %        d31 d32 0 d34
    %        d41 d42 d43 0]
    d_aux = zeros(number_of_points, number_of_points);
    
    for kk = 1:number_of_points
        for hh = kk+1:number_of_points
            dist =norm(points(:,kk) - points(:,hh));
            %Take norm between two different points and add noise
            
            if(dist>disk_radius)%out of range, there is no edge between them
                d=0;
            else
                d = dist + range_deviation*randn();
                if d <= 0
                    d = 1e-5; % to avoid issues in the algorithm
                end
            end
            d_aux(hh,kk) = d;
            d_aux(kk,hh) = d;
        end
    end
    distance_matrices_stack = [distance_matrices_stack d_aux];
end
end

