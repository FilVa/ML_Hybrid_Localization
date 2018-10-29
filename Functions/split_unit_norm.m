function [v,V] = split_unit_norm(vector,dim)
%UNTITLED3 Summary of this function goes here
%   Splits vector 'vector' into a stack of unit norm vectors and magnitudes

n_pos = length(vector)/dim;
v = zeros(length(vector),1);
V = zeros(length(vector),1);

    for i=1:n_pos
        ind_begin = (i-1)*dim+1;
        ind_end = i*dim;
        pos = vector(ind_begin:ind_end);
        if(norm(pos)==0)
            v(ind_begin:ind_end)= zeros(dim,1);
        else
            v(ind_begin:ind_end) = pos/norm(pos);
        end
        V(ind_begin:ind_end) = norm(pos).*ones(dim,1);
    end
end

