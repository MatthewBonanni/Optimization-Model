%OPTIMIZE
% 
% Script to implement gradient ascent
% 
% Matthew R. Bonanni
% 05-2017

%% Configuration

model_step_count = 30;
seed_count = 5;

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

%Objectives
pareto_obj = struct('cam_moment_ratio',     [], ...
                    'beam_length',          [], ...
                    'button_travel',        []  ...
                    );

objectives_fields = fieldnames(pareto_obj);

for i = 1:numel(objectives_fields)
    pareto_obj.(objectives_fields{i}) = pareto_objective;
end

%Weights
pareto_obj.cam_moment_ratio.weight =    1;
pareto_obj.beam_length.weight =         -10;
pareto_obj.button_travel.weight =       -1;

%Create seed points
seeds = randi(model_step_count, [seed_count, numel(indeps_fields)]);

%Create switch array for surrounding values
nearby = double(dec2bin(0:(2^numel(indeps_fields)-1))=='1');
nearby(nearby == 0) = -1;

%% Find utopia points

%Variables for keeping track of maxima
obj_border =        0;
obj_border_max =    0;
obj_max =           zeros(seed_count,1);

%Variables for keeping track of position
posn = [];
border_posn = [];

%Iterate over objectives
for i = 1:length(objectives_fields)
    
    %Iterate over seeds
    for j = 1:size(seeds, 1)
        
        peak = false;
        
        posn = seeds(j,:);
        
        %Fill indeps struct with seed values
        for k = 1:numel(indeps_fields)
            indeps.(indeps_fields{k}) = feasible.(indeps_fields{k})(posn(k));
        end
        
        %Get initial objective values for comparison
        objectives = eject_model(indeps, params, consts);
        obj_max(j) = mean(objectives.(objectives_fields{i}));
        
        
        
        while ~peak
            
            obj_border_max = obj_max(j);
            
            for k = 1:size(nearby,1)
                
                %Go to next border position
                border_posn = posn + nearby(k,:);
                if any(border_posn < 1) || any(border_posn > model_step_count)
                    continue;
                end
                
                %Fill indeps struct with correct parameter values
                for l = 1:numel(indeps_fields)
                    indeps.(indeps_fields{l}) = feasible.(indeps_fields{l})(border_posn(l));
                end
                
                %Run model
                objectives = eject_model(indeps, params, consts);
                obj_border = mean(objectives.(objectives_fields{i}));
                
                %Check that outputs are valid
                if ~isreal(obj_border)
                    continue;
                end
                
                %If this the new best border point, write it to obj_border_max
                if obj_border > obj_border_max
                    obj_border_max = obj_border;
                    border_max_posn = border_posn;
                end
            end
            
            %Move the cursor to the highest scored border point
            if obj_border_max > obj_max(j)
                posn = border_max_posn;
                obj_max(j) = obj_border_max;
            else
                peak = true;
            end
        end
        
        disp(obj_max)
        
    end
    
    %Write utopia point to optimization struct
    pareto_obj.(objectives_fields{i}).utopia = mean(obj_max);
    
    %Reset max values
    obj_border =        0;
    obj_border_max =    0;
    obj_max =           zeros(seed_count,1);
    
end

%% Find Nadir points

%% Find Pareto optimal solution

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