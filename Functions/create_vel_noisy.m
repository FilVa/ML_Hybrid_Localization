function [ vel_nodes_n ] = create_vel_noisy(vel_nodes,dim,heading_deviation,speed_deviation)
%create_vel_noisy Creates matrices with velocity measurements for each
%vehicle with additional noise: Gaussian for speed and von Mises-Fisher for
%heading.
%   Input
%   - vel_nodes : nodes velocities for each time step
%   - dim : dimension
%   - heading_deviation : concentration parameter for noise in heading
%   - speed_deviation : standard deviation for noise in speed
%   Output
%   - vel_nodes_n : noisy velocity for each time step (over columns) and each node (over rows). 

sz= size(vel_nodes);
n_iter = sz(2);
unknowns = sz(1)/dim;

vel_nodes_n = zeros(size(vel_nodes));

for ii = 1:n_iter
    for jj = 1:unknowns
        % Take each velocity term
        value = vel_nodes((jj-1)*dim + 1:jj*dim,ii);
        % split into speed and heading. Add respective noise.
        norm_noise = norm(value) + speed_deviation*randn();
        angle = atan2(value(2),value(1))+vmrand(0,heading_deviation);
        
        if(dim==2) % 2D case
            noisy_speed = [norm_noise*cos(angle);norm_noise*sin(angle)];
            
        elseif(dim==3) % 3D case
            d_xy = norm(value(1:2));
            angle_z = atan2(value(3),d_xy)+vmrand(0,heading_deviation);
            noisy_speed = [norm_noise*cos(angle)*cos(angle_z);norm_noise*sin(angle)*cos(angle_z);norm_noise*sin(angle_z)];
        end
        % store value
        vel_nodes_n((jj-1)*dim + 1:jj*dim,ii) = noisy_speed;
    end
end



end

