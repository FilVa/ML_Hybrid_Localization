function [ anchors, vel_anchors, pos_nodes, vel_nodes,dimension,number_of_anchors, n_nodes, time_step] = trajectory_generate( type )
% trajectory_generate Produces anchor and node positions and velocities for each kind
% of trajectory.
%   Input
%   - type : 'laps','lawnmower','helix' (trajectory chosen)
%
%   Output
%   - anchors : anchors positions vector
%   - vel_anchors : anchors velocity vector
%   - pos_nodes : nodes positions vector
%   - vel_nodes : nodes velocity vector
%   - dimension : dimension of R space
%   - number_of_anchors : number of anchors
%   - n_nodes : number of nodes
%   - time_step : time interval between each consecutive position


% Trajectory Types
switch type
    
    case 'laps'
        % Settings
        number_of_anchors=2;
        n_nodes = 2;
        dimension=2;
        rl = 43;
        time_step = 1;
        rr = 29; 
        separation = 8;
        initial_pos = [50;70];
        offset1 = [8;8];
        offset2 = [8;-8];
        
        % Produce Trajectory
        [pos_acc, vel_acc] = trajectoryLaps(rr, rl, separation, ...
            initial_pos, offset1, offset2, time_step);
        initial_pos2 = initial_pos + [16;0];
        [pos_acc2, vel_acc2] = trajectoryLaps( rr, rl, separation, ...
            initial_pos2, offset1, offset2, time_step);
        % Anchors Positions, Velocities
        anchor2 = pos_acc2(1:dimension, :);
        vel2 = vel_acc2(1:dimension,:);
        anchors = [pos_acc(1:dimension,:); anchor2];
        vel_anchors = [vel_acc(1:dimension*(number_of_anchors-1),:); vel2];
        % Nodes Positions, Velocities
        pos_nodes = pos_acc(dimension+1:end,:);
        vel_nodes = vel_acc(dimension +1 :end,:);
        
        
    case 'lawnmower'
        % Settings
        number_of_anchors=2;
        dimension=2;
        n_nodes = 2;
        rl = 29;
        rr = 29; 
        separation = 8;
        initial_pos = [50;70];
        offset1 = [8;8];
        offset2 = [8;-8];
        time_step = 1;
        % Produce Trajectory
        [pos_acc, vel_acc] = trajectoryLawnMower( rr, rl, separation, ...
            initial_pos, offset1, offset2, time_step);
        initial_pos2 = initial_pos + [16;0];
        [pos_acc2, vel_acc2] = trajectoryLawnMower( rr, rl, separation, ...
            initial_pos2, offset1, offset2, time_step);   
        % Anchors Positions, Velocities
        anchor2 = pos_acc2(1:dimension, :);
        vel2 = vel_acc2(1:dimension,:);
        anchors = [pos_acc(1:dimension,:); anchor2];
        vel_anchors = [vel_acc(1:dimension*(number_of_anchors-1),:); vel2];
        % Nodes Positions Velocities
        pos_nodes = pos_acc(dimension+1:end,:); % get only nodes positions
        vel_nodes = vel_acc(dimension+1 :end,:);
        
    case 'helix'
        % Settings
        dimension=3;
        number_of_anchors = 3;
        n_nodes = 2;
        initial_z = 1;
        initial_phase = 0;
        time_step = 1;
        % Produce Trajectory
        [pos_acc, vel_acc] = trajectoryHelix(initial_z,initial_phase); %3 first position is center
        initial_phase = 0.2;
        [pos_acc2, vel_acc2] = trajectoryHelix(initial_z,initial_phase); %3 first position is center
        initial_z = 10;
        initial_phase = 0;
        [pos_acc3, vel_acc3] = trajectoryHelix(initial_z,initial_phase); %3 first position is center
        % Anchors Positions, Velocities
        anchor2 = pos_acc2(1:dimension, :);
        vel2 = vel_acc2(1:dimension,:);
        anchor3 = pos_acc3(1:dimension, :);
        vel3 = vel_acc3(1:dimension,:);
        anchors = [pos_acc(1:dimension,:); anchor2; anchor3];
        vel_anchors = [vel_acc(1:dimension,:); vel2; vel3];
        % Nodes Positions Velocities
        pos_nodes = pos_acc(dimension+1:end,:); 
        vel_nodes = vel_acc(dimension+1 :end,:);        
        
end

end

% Lap Trajectory Function
function [pos_acc, vel_acc] = trajectoryLaps(rr, rl, separation, ...
                initial_pos, offset1, offset2, time_step)
% trajectoryLaps : computes lap trajectories for 3 different vehicles, returning both
% positions and velocities
%   Input :
%   - rr : right radius
%   - rl : left radius
%   - separation : separation between vehicles
%   - initial_pos : initial position
%   - offset1 : offset for second vehicle
%   - offset2 : offset for third vehicle
%   Output
%   - pos_acc : position matrix. Each column corresponds to one time step,
%   containing concatenated vectors of 3 vehicles positions.
%   - vel_acc : velocity matrix

pos_acc = [];
vel_acc = [];

n_iter = 450;

% Initial configuration
previous_position = initial_pos;
previous_mode = 1;

previous_center_of_rotation = 0;
previous_delta_angle = 0;

radius_left = rl;
radius_right = rr;


rl2 = rl + separation;
rr2 = rr + separation;

rl3 = rl - separation;
rr3 = rr - separation;


p2 = initial_pos + offset1;
p3 = initial_pos + offset2;

m2 = 1;
pcr2 = 0;
pda2 = 0;
m3 = 1;
pcr3 = 0;
pda3 = 0;
original_s = 1;

for ii = 1:n_iter
    
    %Compute next values for the 3 nodes
    [position, mode, velocity, center_of_rotation, delta_angle] = ...
        movementDataLaps(previous_position, previous_mode, previous_center_of_rotation, ...
        previous_delta_angle, time_step, radius_left, radius_right, radius_left, radius_right,original_s);
    [p2, m2, v2, pcr2, pda2] =  movementDataLaps(p2, m2, pcr2, pda2, time_step, rl2, rr2, radius_left, radius_right , original_s);
    [p3, m3, v3, pcr3, pda3] =  movementDataLaps(p3, m3, pcr3, pda3, time_step, rl3, rr3, radius_left, radius_right, original_s);
    
    %Update velocity and position
    vel_acc = [vel_acc,[ velocity; v2; v3]];
    pos_acc = [pos_acc, [position; p2;p3]];
    
    %update 'previous values'
    previous_position = position;
    previous_mode = mode;
    previous_center_of_rotation = center_of_rotation;
    previous_delta_angle = delta_angle;
    
end
end

function [position, mode, velocity, center_of_rotation, delta_angle] =...
 movementDataLaps(previous_position, previous_mode, previous_center_of_rotation, ...
 previous_delta_angle, time_step, radius_left, radius_right, ref_left,ref_right, original_s)

% movementDataLaps : computes next position, in order to obtain a lap
% trajectory. The following rules are used to produce such trajectory:

% center_of_rotation is only used if the next mode was rotation, previous_center_of_rotation is only used 
% in case the previous mode was rotation 
% 4 modes:
% 1 - going left
% 2 - going right
% 3 - turn counterclockwise
% 4 - turn clockwise ///also counterclockwise, but up

% bounds: we can only move left until 0.3, move right until 0.75
% the radii for counterclockwise mode is 0.2, for clockwise is 0.15
% the turn modes can only turn at an maximum of half a circumference
% the speed is 0.05 units/sec
% we can only move on the following order: 1 - 3 - 2 - 4 - 1
% only one transition is permitted! else the code fails

%If we stay inside bounds we keep going in that direction, else we change
%with following rules.
%If we change bounds and order of movement we create a different trajectory


% --- Define --- %
left_mode = 1;
right_mode = 2;
counterclockwise_mode = 3;
clockwise_mode = 4;

bound_left = 0;
bound_right = 100;
speed = original_s;

v_rot_coeff_r = radius_right / ref_right;
v_rot_coeff_l = radius_left / ref_left;

switch previous_mode
    
    case left_mode %reduce x position
        
        %if previous x position > 0 : OK
        if previous_position(1) - speed*time_step > bound_left    

            position = [previous_position(1) - speed*time_step; previous_position(2)]; %update x position
            mode = previous_mode; % we keep going left
            center_of_rotation = 0; 
            velocity = (position-previous_position)/time_step; 
            delta_angle = 0; % random number
            
        else %out of bounds : rotate conterclockwise
        
            mode = counterclockwise_mode;
            extra_time = time_step - (previous_position(1) - bound_left)/speed; % the time left for rotation after we reached the bound
            speed = v_rot_coeff_l * speed;                                                                    
            center_of_rotation = [bound_left; previous_position(2) - radius_left];
            omega = speed/radius_left; %rotation speed
            delta_angle = omega * extra_time; %traversed angle
            position = [center_of_rotation(1) - radius_left*sin(delta_angle); center_of_rotation(2) + radius_left*cos(delta_angle)];
            velocity = (position-previous_position)/time_step;
        end   
        
    case right_mode
        if previous_position(1) + speed*time_step < bound_right % in bounds
            position = [previous_position(1) + speed*time_step; previous_position(2)]; 
            mode = previous_mode; % we keep going left
            center_of_rotation = 0; % we don't need, it's a random number
            velocity = (position-previous_position)/time_step;
            delta_angle = 0; % random number
        else
            
            mode = clockwise_mode;
            extra_time = time_step - (bound_right - previous_position(1))/speed; % the time left after we reached the bound                                                                  
            speed = v_rot_coeff_r * speed;
            center_of_rotation = [bound_right; previous_position(2) + radius_right];
            omega = speed/radius_right; %rotation speed
            delta_angle = omega * extra_time; %traversed angle
            position = [center_of_rotation(1) + radius_right*sin(delta_angle); center_of_rotation(2) - radius_right*cos(delta_angle)];
            velocity = (position-previous_position)/time_step;
        end 
    
    case counterclockwise_mode
        speed = v_rot_coeff_l * speed;
        omega = speed/radius_left;
        if previous_delta_angle + time_step*omega < pi
            mode = previous_mode;
            delta_angle = previous_delta_angle + time_step*omega;
            center_of_rotation = previous_center_of_rotation;
            position = [center_of_rotation(1) - radius_left*sin(delta_angle); center_of_rotation(2) + radius_left*cos(delta_angle)];
            velocity = (position-previous_position)/time_step;
        else
            
            extra_time = (previous_delta_angle + time_step*omega - pi)/omega;
            mode = right_mode;
            center_of_rotation = 0;
            speed = original_s;
            position = [previous_center_of_rotation(1) + extra_time*speed; previous_center_of_rotation(2) - radius_left];
            velocity = (position-previous_position)/time_step;
            delta_angle = 0;        
        end
        
        
    case clockwise_mode
        speed = v_rot_coeff_r * speed;
        omega = speed/radius_right;
        if previous_delta_angle + time_step*omega < pi
            mode = previous_mode;
            delta_angle = previous_delta_angle + time_step*omega;
            center_of_rotation = previous_center_of_rotation;
            position = [center_of_rotation(1) + radius_right*sin(delta_angle); center_of_rotation(2) - radius_right*cos(delta_angle)];
            velocity = (position-previous_position)/time_step;
        else
            
            extra_time = (previous_delta_angle + time_step*omega - pi)/omega;
            mode = left_mode;
            center_of_rotation = 0;
            speed = original_s;
            position = [previous_center_of_rotation(1) - extra_time*speed; previous_center_of_rotation(2) + radius_right];
            velocity = (position-previous_position)/time_step;
            delta_angle = 0;        
        end
end 
end


function [pos_acc, vel_acc] = trajectoryLawnMower(rr, rl, separation, ...
                initial_pos, offset1, offset2, time_step)
% trajectoryLawnmower : computes lawnmower trajectories for 3 different vehicles, returning both
% positions and velocities
%   Input :
%   - rr : right radius
%   - rl : left radius
%   - separation : separation between vehicles
%   - initial_pos : initial position
%   - offset1 : offset for second vehicle
%   - offset2 : offset for third vehicle
%   Output
%   - pos_acc : position matrix. Each column corresponds to one time step,
%   containing concatenated vectors of 3 vehicles positions.
%   - vel_acc : velocity matrix
            
pos_acc = [];
vel_acc = [];
n_iter = 800;

%initial config:
previous_position = initial_pos;
previous_mode = 1;

previous_center_of_rotation = 0;
previous_delta_angle = 0;
vel_acc = [];
radius_left = rl;
radius_right = rr;

rl2 = rl + separation;
rr2 = rr - separation; 

rl3 = rl - separation;
rr3 = rr + separation;

p2 = initial_pos + offset1;
p3 = initial_pos + offset2;

m2 = 1;
pcr2 = 0;
pda2 = 0;
m3 = 1;
pcr3 = 0;
pda3 = 0;
original_s = 1;

for ii = 1:n_iter
    
    %Compute next values for the 3 nodes
    [position, mode, velocity, center_of_rotation, delta_angle] = ...
    movementDataLawnMower(previous_position, previous_mode, previous_center_of_rotation, ...
        previous_delta_angle, time_step, radius_left, radius_right, radius_left, radius_right,original_s);
    [p2, m2, v2, pcr2, pda2] =  movementDataLawnMower(p2, m2, pcr2, pda2, time_step, rl2, rr2, radius_left, radius_right , original_s);
    [p3, m3, v3, pcr3, pda3] =  movementDataLawnMower(p3, m3, pcr3, pda3, time_step, rl3, rr3, radius_left, radius_right, original_s);
    
    %Update velocity and position
    vel_acc = [vel_acc,[ velocity; v2; v3]];
    pos_acc = [pos_acc, [position; p2;p3]];
    
    %update 'previous values'
    previous_position = position;
    previous_mode = mode;
    previous_center_of_rotation = center_of_rotation;
    previous_delta_angle = delta_angle;
    

end
end

function [position, mode, velocity, center_of_rotation, delta_angle] =...
 movementDataLawnMower(previous_position, previous_mode, previous_center_of_rotation, ...
 previous_delta_angle, time_step, radius_left, radius_right, ref_left,ref_right, original_s)
% movementDataLawnMower : computes next position, in order to obtain a
% lawnmower trajectory. The following rules are used to produce such trajectory:

% center_of_rotation is only used if the next mode was rotation, previous_center_of_rotation is only used 
% in case the previous mode was rotation 
% 4 modes:
% 1 - going left
% 2 - going right
% 3 - turn counterclockwise
% 4 - turn clockwise ///also counterclockwise, but up

% bounds: we can only move left until 0.3, move right until 0.75
% the radii for counterclockwise mode is 0.2, for clockwise is 0.15
% the turn modes can only turn at an maximum of half a circumference
% the speed is 0.05 units/sec
% we can only move on the following order: 1 - 3 - 2 - 4 - 1
% only one transition is permitted! else the code fails

%If we stay inside bounds we keep going in that direction, else we change
%with following rules.
%If we change bounds and order of movement we create a different trajectory


% --- Define --- %
left_mode = 1;
right_mode = 2;
counterclockwise_mode = 3;
clockwise_mode = 4;

bound_left = 0;%0
bound_right = 100;%100

speed = original_s;

v_rot_coeff_r = radius_right / ref_right;
v_rot_coeff_l = radius_left / ref_left;

switch previous_mode
    
    case left_mode %reduce x position
        
        %if previous x position > 0 : OK
        if previous_position(1) - speed*time_step > bound_left    

            position = [previous_position(1) - speed*time_step; previous_position(2)]; %update x position
            mode = previous_mode; % we keep going left
            center_of_rotation = 0; % we don't need, it's a random number
            velocity = (position-previous_position)/time_step; %DUVIDA : não é igual a speed?
            delta_angle = 0; % random number
            
        else %out of bounds : rotate conterclockwise
        
            mode = counterclockwise_mode;
            extra_time = time_step - (previous_position(1) - bound_left)/speed; % the time left for rotation
            speed = v_rot_coeff_l * speed;                                                                    % after we reached the bound
            center_of_rotation = [bound_left; previous_position(2) - radius_left];
            omega = speed/radius_left; %rotation speed
            delta_angle = omega * extra_time; %traversed angle
            position = [center_of_rotation(1) - radius_left*sin(delta_angle); center_of_rotation(2) + radius_left*cos(delta_angle)];
            velocity = (position-previous_position)/time_step;
        end   
        
    case right_mode
        if previous_position(1) + speed*time_step < bound_right % in bounds
            position = [previous_position(1) + speed*time_step; previous_position(2)]; 
            mode = previous_mode; % we keep going left
            center_of_rotation = 0; % we don't need, it's a random number
            velocity = (position-previous_position)/time_step;
            delta_angle = 0; % random number
        else
            
            mode = clockwise_mode;
            extra_time = time_step - (bound_right - previous_position(1))/speed; % the time left
                                                                              % after we reached the bound
            speed = v_rot_coeff_r * speed;
            center_of_rotation = [bound_right; previous_position(2) - radius_right];
            omega = speed/radius_right; %rotation speed
            delta_angle = omega * extra_time; %traversed angle
            position = [center_of_rotation(1) + radius_right*sin(delta_angle); center_of_rotation(2) + radius_right*cos(delta_angle)];
            velocity = (position-previous_position)/time_step;
        end 
    
    case counterclockwise_mode
        speed = v_rot_coeff_l * speed;
        omega = speed/radius_left;
        if previous_delta_angle + time_step*omega < pi
            mode = previous_mode;
            delta_angle = previous_delta_angle + time_step*omega;
            center_of_rotation = previous_center_of_rotation;
            position = [center_of_rotation(1) - radius_left*sin(delta_angle); center_of_rotation(2) + radius_left*cos(delta_angle)];
            velocity = (position-previous_position)/time_step;
        else
            
            extra_time = (previous_delta_angle + time_step*omega - pi)/omega;
            mode = right_mode;
            center_of_rotation = 0;
            speed = original_s;
            position = [previous_center_of_rotation(1) + extra_time*speed; previous_center_of_rotation(2) - radius_left];
            velocity = (position-previous_position)/time_step;
            delta_angle = 0;        
        end
        
        
    case clockwise_mode
        speed = v_rot_coeff_r * speed;
        omega = speed/radius_right;
        if previous_delta_angle + time_step*omega < pi
            mode = previous_mode;
            delta_angle = previous_delta_angle + time_step*omega;
            center_of_rotation = previous_center_of_rotation;
            position = [center_of_rotation(1) + radius_right*sin(delta_angle); center_of_rotation(2) + radius_right*cos(delta_angle)];
            velocity = (position-previous_position)/time_step;
        else
            
            extra_time = (previous_delta_angle + time_step*omega - pi)/omega;
            mode = left_mode;
            center_of_rotation = 0;
            speed = original_s;
            position = [previous_center_of_rotation(1) - extra_time*speed; previous_center_of_rotation(2) - radius_right];
            velocity = (position-previous_position)/time_step;
            delta_angle = 0;        
        end
end 
end



function [pos_acc, vel_acc] = trajectoryHelix(initial_z,initial_phase)
% trajectoryHelix produces helix trajectory for 3 nodes
%   Input :
%   - initial_z : initial position in z axis
%   - initial_phase : initial angle in xy axis
%   Output : 
%   - pos_acc : position matrix. Each column corresponds to one time step,
%   containing concatenated vectors of 3 vehicles positions.
%   - vel_acc : velocity matrix.

% Settings
n_iter = 500;
time_step = 1;
speed_2d = 0.8;
speed_z = -0.1;
radius = 20;
center_2d = [0; 0];
mode = 1;
offset2d = 5;
offsetz = 5;

omega = speed_2d/radius;
previous_delta_angle = initial_phase;
center_of_rotation = center_2d;
previous_z = initial_z;
radius2 =radius +offset2d;
radius3 = radius -offset2d;
previous_z_nodes = initial_z - offsetz;

previous_position = [center_of_rotation(1) + radius*sin(previous_delta_angle); center_of_rotation(2) - radius*cos(previous_delta_angle); previous_z];
previous_position_2 = [center_of_rotation(1) + radius2*sin(previous_delta_angle); center_of_rotation(2) - radius2*cos(previous_delta_angle); previous_z_nodes];
previous_position_3 = [center_of_rotation(1) + radius3*sin(previous_delta_angle); center_of_rotation(2) - radius3*cos(previous_delta_angle); previous_z_nodes];
pos_acc=[];
vel_acc=[];

for ii = 1:n_iter
    
    delta_angle = previous_delta_angle + time_step*omega;
    new_z = previous_z + time_step*speed_z;
    new_z_nodes = previous_z_nodes + time_step*speed_z;
    
    % position for each node
    position = [center_of_rotation(1) + radius*sin(delta_angle); center_of_rotation(2) - radius*cos(delta_angle); new_z];
    position2 = [center_of_rotation(1) + radius2*sin(delta_angle); center_of_rotation(2) - radius2*cos(delta_angle); new_z_nodes];
    position3 =  [center_of_rotation(1) + radius3*sin(delta_angle); center_of_rotation(2) - radius3*cos(delta_angle); new_z_nodes];
    
    % velocity for each node
    velocity = (position-previous_position)/time_step;
    velocity2 = (position2-previous_position_2)/time_step;
    velocity3 = (position3-previous_position_3)/time_step;
    
    % update previous values
    previous_z=new_z;
    previous_z_nodes = new_z_nodes;
    previous_delta_angle = delta_angle;
    previous_position = position;
    previous_position_2 = position2;
    previous_position_3 = position3;
    
    % update velocity and position vectors
    pos_acc =[pos_acc, [position;position2;position3]];
    vel_acc = [vel_acc, [velocity;velocity2;velocity3]];
end
    

end

