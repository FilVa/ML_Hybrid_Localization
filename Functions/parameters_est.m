function [ param_est,new_stack ] = parameters_est(i,type,flag,fix_value,dim,time_window,number_edges,old_stack,measure_vec,estimated_vec)
%parameters_est estimates parameter for either std of gaussian distribution
%or concentration parameter of vMF
%   Input
%   - i : current iteration
%   - type : either 'sigma' for Gaussian distribution or 'kappa' for vMF
%   - flag : use fix value (flag=0) or estimate (flag!=0)
%   - dim : dimension
%   - time_window
%   - number_edges : number of edges. each edges has one estimated
%   parameter
%   - old_stack : stores previous computation to avoid redoing them
%   - measure_vec : current obtained measurement
%   - estimated_vec : estimated measurement obtained by our algorithm


if(flag==0) %use fix value
    param_est = repmat(fix_value,number_edges*dim*time_window,1);
    new_stack=old_stack;% not used
else
    if(strcmp(type,'sigma')) % Gaussian 
        new_stack = get_stack_std(old_stack,measure_vec,estimated_vec); %update stack
        param_est = compute_sigma(new_stack,dim,time_window,i); % estimate parameter
        
    elseif(strcmp(type,'kappa'))% von Mises-Fisher
        new_stack = get_stack_kappa(old_stack,measure_vec,estimated_vec);%update stack
        param_est = compute_kappa(new_stack,i,dim,time_window);% estimate parameter
    end
    
    % when inf is returned for some estimates (first iterations) replace by default value to avoid errors 
    param_est(isinf(param_est)) = fix_value;
end
end


function [new_stack] = get_stack_std(old_stack,measure_vec,estimated_vec)
%get_stack_std update stack for gaussian distribution

new_stack = zeros(size(old_stack));

for i=1:length(measure_vec)    
    new_term = (measure_vec(i)-estimated_vec(i))^2;
    new_stack(i) = old_stack(i) + new_term;
end

end



function [new_stack] = get_stack_kappa(old_stack,measure_vec,estimated_vec)
%get_stack_kappa update stack for vMF

% Initialize stack
new_stack = zeros(size(old_stack));
dim = length(old_stack)/size(measure_vec,1);

for i=1:size(measure_vec,1)
   
    dif_angle = angdiff(estimated_vec(i,:) ,measure_vec(i,:)); % angle difference
 
    % New component
    if(dim==2)
        new_term =[cos(dif_angle );sin(dif_angle )];
    elseif(dim==3)
        angle= dif_angle(1);
        angle_z=dif_angle(2);
        new_term = [cos(angle)*cos(angle_z);sin(angle)*cos(angle_z);sin(angle_z)];
    end
    
    % Add to stack vector
    new_stack((i-1)*dim+1:i*dim,1) = old_stack((i-1)*dim+1:i*dim,1) + new_term;

end

end


function [kappa_est] = compute_kappa(stack,t,dim,time_window)
%compute_kappa parameter estimation for vMF

kappa_est_single = zeros(length(stack)/dim,1);

for edge=1:length(stack)/dim
    ind_begin = (edge-1)*dim+1;
    ind_end = edge*dim;
    
    r_bar = norm(stack(ind_begin:ind_end))/t;
    
    % Banerjee
    k_bar = r_bar*(dim-r_bar^2)/(1-r_bar^2);
   
    if( k_bar>10^4 || k_bar<0)%fix nan
         k_bar=inf;
    end

    kappa_est_single(edge) = k_bar;
end

% expand for dim
kappa_est_dim = repelem(kappa_est_single,dim,1);

%expand for time window
kappa_est = repmat(kappa_est_dim,time_window,1);

end

function [sigma_est] = compute_sigma(stack,dim,time_window,i)
%compute_sigma parameter estimation for gaussian

stack = stack.*(1/(i)); %division by time
stack = stack.^(1/2); %square root, we need sigma
stack = stack.^(-1); %we need 1/sigma

% expand for dim
stack_dim = repelem(stack,dim,1);
%expand for time window
sigma_est = repmat(stack_dim,time_window,1);

end



