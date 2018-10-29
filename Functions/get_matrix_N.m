function [N] = get_matrix_N(n_nodes,dim,time_window)
%get_matrix_N Matrix to produce x(t)-x(t-1) when multiplied with nodes
%vector
%   Input
%   - n_nodes : number of nodes
%   - dim : dimension
%   - time_window : current time window
%   Output
%   - N

N = zeros(n_nodes*dim*time_window);

if(time_window>1)
    for i=2:time_window
        rows_begin = n_nodes*dim*(i-1)+1;
        rows_end = rows_begin+n_nodes*dim-1;
        col_begin =  n_nodes*dim*(i-2)+1;
        col_end = col_begin -1 +n_nodes*dim;
        
        % previous
        N(rows_begin:rows_end,col_begin:col_end) = -1.*eye(n_nodes*dim);
        %current
        N(rows_begin:rows_end,rows_begin:rows_end) = 1.*eye(n_nodes*dim);
              
    end
end

end

