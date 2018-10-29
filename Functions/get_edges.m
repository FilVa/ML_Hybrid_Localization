function [ edge_array_nodes, australian_edges,number_edges_nodes,number_edges_anchors] = get_edges( distances,n_anchors,n_points )
%get_edges Get edges between two nodes and between node-anchor.
%   Input
%   - distances : matrix storing ranges between node-node, node-anchor
%   - n_anchors : number of anchors
%   - n_points : number of anchors+nodes
%   Output
%   - edge_array_nodes : pairs of indices for each edge node-node
%   - australian_edges : pairs of indices for each edge node-anchor
%   - number_edges_nodes : number of edges node-node
%   - number_edges_anchors : number of edges node-anchor

    number_of_anchors = n_anchors;
    check_matrix = distances(:, 1:n_points) > 0; % where do we have edges
    
    edge_array = [];
    
    %counter = 0; % this is to separate the different type of edges, node-node, node-edge
    for ii = 1:n_points
        for jj = ii + 1:n_points
            if check_matrix(ii,jj)
                edge_array = [edge_array; [ii, jj]];
            end
        end
    end
    
    edge_array_nodes = []; %only between unknown nodes
    edge_array_nodes_anchors = []; % only between nodes and anchors, the unknowns are always the second index
    edge_array_anc_anc = [];
    
    for ii = 1:size(edge_array,1) 
        if edge_array(ii,1) > number_of_anchors && edge_array(ii,2) > number_of_anchors
            %first and second indices of edge refer to a node
            edge_array_nodes = [edge_array_nodes; edge_array(ii,:)];
        
        
        % if one is from node and the other from anchor
        elseif edge_array(ii,1) <= number_of_anchors && edge_array(ii,2) > number_of_anchors
            edge_array_nodes_anchors = [edge_array_nodes_anchors; edge_array(ii,:)];
        
        elseif edge_array(ii,1) > number_of_anchors && edge_array(ii,2) <= number_of_anchors
            edge_array_nodes_anchors = [edge_array_nodes_anchors; edge_array(ii,:)];
        
        end
        
        %disregard edges between anchor and anchor
    end
    
    %  our edges with anchors-nodes are not the most suitable to continue the algorithm, 
    % australian_edges stack them according to node index, opposed to anchor index
    n_nodes = n_points - n_anchors;
    australian_edges = get_australian_edges(edge_array_nodes_anchors,n_anchors,n_nodes);
    
    number_edges_anchors = size(australian_edges, 1);
    number_edges_nodes = size(edge_array_nodes, 1);
    
end

function [ australian_edges ] = get_australian_edges(edge_array_nodes_anchors,number_of_anchors,unknowns)
%get_australian_edges Order edges by nodes instead of anchors
    australian_edges = [];
    for jj = 1:unknowns
        for ii = 1:size(edge_array_nodes_anchors,1)
            indexn = edge_array_nodes_anchors(ii,2) - number_of_anchors;
            
            if indexn == jj
                australian_edges = [australian_edges; edge_array_nodes_anchors(ii,:)];
            end
        end
    end

end


