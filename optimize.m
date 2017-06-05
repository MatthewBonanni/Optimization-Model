%OPTIMIZE
% 
% Script to implement gradient ascent
% 
% Matthew R. Bonanni
% 06-2017

%% Configuration

model_step_count = 100;
seed_count = 10;

%Physical parameters set in stone
consts = struct('lever_sweep',      25, ...
                'hinge_x',          53.25, ...
                'button_to_lever',  19 ...
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

%Create objectives struct for optimization
pareto_obj = struct('cam_moment_ratio',     [], ...
                    'beam_length',          [], ...
                    'button_travel',        []  ...
                    );

%Fill objectives struct with optimization objects
obj_fields = fieldnames(pareto_obj);
for i = 1:numel(obj_fields)
    pareto_obj.(obj_fields{i}) = pareto_objective;
end

%Weights
pareto_obj.cam_moment_ratio.weight =    2;
pareto_obj.beam_length.weight =        -5;
pareto_obj.button_travel.weight =      -1;

%Create seed points
seeds = randi(model_step_count, [seed_count, numel(indeps_fields)]);

%Create switch array for surrounding values
nearby = double(dec2bin(0:(2^numel(indeps_fields)-1))=='1');
nearby(nearby == 0) = -1;

%% Find utopia and Nadir points

%Variables for keeping track of maxima/minima
obj_border =         0;
obj_border_best =    0;
obj_best =           zeros(seed_count,1);

%Variables for keeping track of cursor position
posn =              [];
border_posn =       [];
border_best_posn =  [];

%Loop twice (once for utopia, once for Nadir)
for gradient = 1:2
    
    %Iterate over objectives
    for i = 1:length(obj_fields)
        
        %Iterate over seeds
        for seed = 1:size(seeds, 1)
            
            optimal = false;
            
            posn = seeds(seed,:);
            
            %Fill indeps struct with seed values
            for field = 1:numel(indeps_fields)
                indeps.(indeps_fields{field}) = feasible.(indeps_fields{field})(posn(field));
            end
            
            %Get initial objective values for comparison
            objectives = eject_model(indeps, params, consts);
            obj_best(seed) = mean(objectives.(obj_fields{i}));
            
            %Ensure seed is valid
            if ~isreal(obj_best(seed))
                obj_best(seed) = NaN;
                continue;
            elseif obj_best(seed) < 0
                obj_best(seed) = NaN;
                continue;
            end
            
            while ~optimal
                
                obj_border_best = obj_best(seed);
                
                %Iterate over bordering parameter sets
                for border_pt = 1:size(nearby,1)
                    
                    %Go to next border position
                    border_posn = posn + nearby(border_pt,:);
                    if any(border_posn < 1) || any(border_posn > model_step_count)
                        continue;
                    end
                    
                    %Fill indeps struct with correct parameter values
                    for field = 1:numel(indeps_fields)
                        indeps.(indeps_fields{field}) = feasible.(indeps_fields{field})(border_posn(field));
                    end
                    
                    %Run model
                    objectives = eject_model(indeps, params, consts);
                    obj_border = mean(objectives.(obj_fields{i}));
                    
                    %Ensure output is valid
                    if ~isreal(obj_border)
                        continue;
                    elseif obj_border < 0
                        continue;
                    end
                    
                    %If this the new best border point, write it to
                    %obj_border_best
                    
                    %Utopia (ascending)
                    if gradient == 1
                        if obj_border > obj_border_best
                            obj_border_best = obj_border;
                            border_best_posn = border_posn;
                        end
                    
                    %Nadir (descending)
                    elseif gradient == 2
                        if obj_border < obj_border_best
                            obj_border_best = obj_border;
                            border_best_posn = border_posn;
                        end
                    end
                end
                
                %Move the cursor to the best scored border point
                
                %Utopia (ascending)
                if gradient == 1
                    if obj_border_best > obj_best(seed)
                        posn = border_best_posn;
                        obj_best(seed) = obj_border_best;
                    else
                        optimal = true;
                    end
                    
                    %Nadir (descending)
                elseif gradient == 2
                    if obj_border_best > obj_best(seed)
                        posn = border_best_posn;
                        obj_best(seed) = obj_border_best;
                    else
                        optimal = true;
                    end
                end
            end
        end
        
        %Write utopia/Nadir point to optimization struct
        if gradient == 1
            pareto_obj.(obj_fields{i}).utopia = mean(obj_best(~isnan(obj_best)));
        elseif gradient == 2
            pareto_obj.(obj_fields{i}).nadir = mean(obj_best(~isnan(obj_best)));
        end
        
        %Reset best values
        obj_border =         0;
        obj_border_best =    0;
        obj_best =           zeros(seed_count,1);
        
    end
end

%% Find Pareto optimal solution

%Normalized objective scores
obj_normal = 0;
score = 0;
pareto_border_best = 0;
pareto_best = zeros(seed_count, 1);

%Locations of optimal solutions
pareto_posn = zeros(size(seeds));

%Iterate over seeds
for seed = 1:size(seeds, 1)
    
    optimal = false;
    
    posn = seeds(seed,:);
    
    %Fill indeps struct with seed values
    for field = 1:numel(indeps_fields)
        indeps.(indeps_fields{field}) = feasible.(indeps_fields{field})(posn(field));
    end
    
    %Get initial objective values for comparison
    objectives = eject_model(indeps, params, consts);
    
    %Tally score
    score = 0;
    for field = 1:numel(obj_fields)
        
        %Get mean of outputs
        objectives.(obj_fields{field}) = mean(objectives.(obj_fields{field}));
        
        %Normalize using formula xnew = (x - xmin)/(xmax - xmin)
        obj_normal = ...
            (objectives.(obj_fields{field}) - pareto_obj.(obj_fields{field}).nadir) / ...
            (pareto_obj.(obj_fields{field}).utopia - pareto_obj.(obj_fields{field}).nadir);
        
        %Multiply by weight, add to score
        score = score + obj_normal * pareto_obj.(obj_fields{field}).weight;
        
    end
    
    pareto_best(seed) = score;
    
    while ~optimal
        
        obj_border_best = obj_best(seed);
        
        %Iterate over bordering parameter sets
        for border_pt = 1:size(nearby,1)
            
            %Go to next border position
            border_posn = posn + nearby(border_pt,:);
            if any(border_posn < 1) || any(border_posn > model_step_count)
                continue;
            end
            
            %Fill indeps struct with correct parameter values
            for field = 1:numel(indeps_fields)
                indeps.(indeps_fields{field}) = feasible.(indeps_fields{field})(border_posn(field));
            end
            
            %Run model
            objectives = eject_model(indeps, params, consts);
            
            %Tally score
            score = 0;
            for field = 1:numel(obj_fields)
                
                %Get mean of outputs
                objectives.(obj_fields{field}) = mean(objectives.(obj_fields{field}));
                
                %Normalize using formula xnew = (x - xmin)/(xmax - xmin)
                obj_normal = ...
                    (objectives.(obj_fields{field}) - pareto_obj.(obj_fields{field}).nadir) / ...
                    (pareto_obj.(obj_fields{field}).utopia - pareto_obj.(obj_fields{field}).nadir);
                
                %Multiply by weight, add to score
                score = score + obj_normal * pareto_obj.(obj_fields{field}).weight;
                
            end
            
            %If this the new best border point, write it to
            %pareto_border_best
            if score > pareto_border_best
                pareto_border_best = score;
                border_best_posn = border_posn;
            end
        end
        
        %Move the cursor to the best scored border point
        if pareto_border_best > pareto_best(seed)
            pareto_posn(seed, :) = border_best_posn;
            pareto_best(seed) = pareto_border_best;
        else
            optimal = true;
        end
    end
end

[optimal_score, i_max] = max(pareto_best);
optimal_posn = pareto_posn(i_max, :);

%Display results

for field = 1:numel(indeps_fields)
    disp([indeps_fields(field), feasible.(indeps_fields{field})(optimal_posn(field))]);
    indeps.(indeps_fields{field}) = feasible.(indeps_fields{field})(optimal_posn(field));
end

objectives = eject_model(indeps, params, consts);

for field = 1:numel(obj_fields)
    objectives.(obj_fields{field}) = mean(objectives.(obj_fields{field}));
    disp([obj_fields(field), objectives.(obj_fields{field})]);
end