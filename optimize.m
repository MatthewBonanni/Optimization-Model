%OPTIMIZE
% 
% Script to implement gradient ascent
% 
% Matthew R. Bonanni
% 05-2017

%% Configuration

model_step_count = 30;
seed_count = 3;

%Physical parameters set in stone
consts = struct('lever_sweep',      25, ...
                'hinge_x',          53.25, ...
                'button_to_lever',  14 ...
               );

%Will probably stay the same
params = struct('cam_end_rad',      2, ...
                'lever_end_rad',    1 ...
               );
%Constraints
max_button_travel = 8;
button_to_wall = 14.93;
max_contact_offset = 3;

%Create struct of feasible parameter space
feasible = struct('button_contact_x_i',   [], ...
                  'button_contact_y',     [], ...
                  'cam_sweep',            [], ...
                  'cam_end_dist',         []  ...
                  );

%Fill parameter vectors
feasible.button_contact_x_i =    linspace(-max_contact_offset, max_contact_offset, model_step_count);
feasible.button_contact_y =      linspace(2, 15, model_step_count);
feasible.cam_sweep =             linspace(0, 90, model_step_count);
feasible.cam_end_dist =          linspace(3, button_to_wall + max_contact_offset, model_step_count);

%Create struct for holding single values to pass to model
indeps = struct('button_contact_x_i',   [], ...
                'button_contact_y',     [], ...
                'cam_sweep',            [], ...
                'cam_end_dist',         []  ...
                );

indeps_fields = fieldnames(indeps);

%Weights
cam_moment_ratio_weight = 1;
beam_length_weight = -10;
button_travel_weight = -1;

%Create seed points
seeds = randi(model_step_count, [seed_count, numel(indeps_fields)]);

%Create switch array for surrounding values
nearby = double(dec2bin(0:(2^numel(indeps_fields)-1))=='1');
nearby(nearby == 0) = -1;

%% Find utopia points
utopia_pos = zeros(numel(indeps_fields), seed_count);

%Iterate over seeds
for i = 1:size(seeds, 1)
    
    peak = false;
    
    posn = seeds(i,:);
    
    %Organize seed inputs
    
    %Fill indeps struct with correct parameter values
    for j = 1:numel(indeps_fields)
        indeps.(indeps_fields{j}) = feasible.(indeps_fields{j})(posn(j));
    end
    
    [cam_moment_ratio, beam_length, button_travel] = eject_model(indeps, params, consts);
    cam_moment_ratio_max = mean(cam_moment_ratio);
    
    while ~peak
        
        cam_moment_ratio_border_max = cam_moment_ratio_max;
        
        for j = 1:size(nearby,1)
            
            border_posn = posn + nearby(j,:);
            if any(border_posn < 1) || any(border_posn > model_step_count)
                continue;
            end                
            
            %Fill indeps struct with correct parameter values
            for k = 1:numel(indeps_fields)
                indeps.(indeps_fields{k}) = feasible.(indeps_fields{k})(border_posn(k));
            end
            
            [cam_moment_ratio, beam_length, button_travel] = eject_model(indeps, params, consts);
            cam_moment_ratio_border_new = mean(cam_moment_ratio);
            
            if cam_moment_ratio_border_new > cam_moment_ratio_border_max
                cam_moment_ratio_border_max = cam_moment_ratio_border_new;
                border_max = border_posn;
            end
        end
        
        if cam_moment_ratio_border_max > cam_moment_ratio_max
            posn = border_max;
            cam_moment_ratio_max = cam_moment_ratio_border_max;
        else
            peak = true;
        end
        
        disp(cam_moment_ratio_max);
        
    end
end

%Iterate over seed points
for i = 1:size(seeds, 1)
    
    peak = false;
    
    %Organize seed inputs
    indeps = struct('button_contact_x_i',   button_contact_x_i( seeds(i,1)), ...
                    'button_contact_y',     button_contact_y(   seeds(i,2)), ...
                    'cam_sweep',            cam_sweep(          seeds(i,3)), ...
                    'cam_end_dist',         cam_end_dist(       seeds(i,4)) ...
                    );
    
    [cam_moment_ratio, beam_length, button_travel] = eject_model(indeps, params, consts);
    
    while ~peak
        
    end
end