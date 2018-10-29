function [ A,A_time ] = get_matrix_A(edge_array_nodes,n_anchors,n_nodes,dim,time_window)
%get_matrix_A Returns arc-node incidence matrix between pairs of nodes extended over dimension and
%over time window.
%   Input
%   - edge_array_nodes : pairs of indices indicating edges between two
%   nodes
%   - n_anchors : number of anchors
%   - n_nodes : number of nodes
%   - dim : dimension
%   - time_window : current time window
%   Output
%   - A : arc-node incidence matrix extended over dimension (kronecker product)
%   - A_time : matrix A extended over time window (kronecker product)


    number_of_edges_nodes = size(edge_array_nodes,1);
    
    %arc-node incidence matrix, only between nodes
    C = zeros(number_of_edges_nodes, n_nodes); 
    for ii = 1:size(edge_array_nodes,1);
        C(ii, edge_array_nodes(ii,1)- n_anchors) = -1;
        C(ii, edge_array_nodes(ii,2)- n_anchors) = 1;    
    end
    
    Id = eye(dim);
    A = kron(C, Id);    
    A_time = kron(eye(time_window),A);

end

