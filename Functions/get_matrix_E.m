function [ X,E] = get_matrix_E( distances, n_anchors,n_nodes,dim,time_window )
%get_matrix_E Returns node selector matrix to match anchor measurements, extended over dimension and
%over time window.
%   Input
%   - distances : matrix with ranges between nodes and anchors
%   - n_anchors : number of anchors
%   - n_nodes : number of nodes
%   - dim : dimension
%   - time_window : current time window
%   Output
%   - X : matrix of 1's and 0's selecting the nodes in correct order to
%   match measurements with anchors (extended over dimension)
%   - E : matrix X extended over time window (kronecker product)

    number_of_points = n_anchors + n_nodes;

    check_matrix = distances(:, 1:number_of_points) > 0; % where do we have measurements
    
    Id = eye(dim);
    
    anchor_connectivity_array = sum(check_matrix(1:n_anchors, n_anchors + 1:end),1); % edges between one anchor and one node
        
    aux = [];
    
    for ii = 1:time_window
        aux = [aux; eye(sum(anchor_connectivity_array))]; 
    end
  
    aux = kron(aux,Id); 
    
    X = zeros(sum(anchor_connectivity_array) ,n_nodes); % the selector
    index_counter = 0;
    
    for ii = 1:n_nodes
        for jj = 1:anchor_connectivity_array(ii)
            X(index_counter + 1, ii) = 1;
            index_counter = index_counter + 1;
        end
    end
    
    X_no_dim = X;
    
    X = kron(X, Id);
    
    E = kron(eye(time_window),X);

end
