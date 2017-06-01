%RUN_MODEL_BRUTEFORCE
% 
% Script to pass large sets of input parameters to the mathematical model
% and gather output data, while filtering physically impossible results.
% 
% Matthew R. Bonanni
% 04-2017

%% Configuration

%Physical parameters set in stone
consts = struct('lever_sweep',      25, ...
                'hinge_x',          53.25, ...
                'button_to_lever',  14 ...
               );

%Will probably stay the same
params = struct('cam_end_rad',      2, ...
                'lever_end_rad',    1 ...
               );

%Independent variables

%Original (P2.0) values:
%button_contact_x_i = 0;
%button_contact_y = 7.42;
%cam_sweep = 30;
%cam_end_dist = 19.6;

%Resolution of input parameters. Caution: iteration count raised to a
%power of 4.
model_step_count = 30;

%Constraints
max_button_travel = 8;
button_to_wall = 14.93;
max_contact_offset = 3;

%Create parameter vectors
button_contact_x_i =    linspace(-max_contact_offset, max_contact_offset, model_step_count);
button_contact_y =      linspace(2, 15, model_step_count);
cam_sweep =             linspace(0, 90, model_step_count);
cam_end_dist =          linspace(3, button_to_wall + max_contact_offset, model_step_count);

%Mark data out of date
model_update = 1;

%% Time estimate of calculation

%Default parameter configuration (P2.0 values)
indeps = struct('button_contact_x_i',   0, ...
                'button_contact_y',     7.42, ...
                'cam_sweep',            30, ...
                'cam_end_dist',         19.6 ...
               );

%Time single calculation
tic;
[cam_moment_ratio, beam_length, button_travel] = ...
    eject_model(indeps, params, consts);
iteration_time = toc;

%Approzimate total calculation time
iteration_count = model_step_count^4;
model_time = iteration_time * iteration_count;
model_time_days = model_time / (60 * 60 * 24);

%Calculation run confirmation dialog
disp(['Estimated model time: ', datestr(model_time_days, 'HH:MM:SS.FFF')]);
prompt = ['Iteration count: ', num2str(iteration_count), char(10), ...
    'Calculation time estimate: ', datestr(model_time_days, 'HH:MM:SS.FFF'), char(10), ...
    'NOTE: Time estimate is imprecise'];
button = questdlg(prompt, 'Time Estimate', 'Continue', 'Quit', 'Continue');
if strcmpi(button, 'Quit')
	return;
end

%Begin timimg actual calculation
tic;

%% Set up output data structure

%optim_data is a 4D cell array of vectors. Each dimension of the array
%represents a different physical input parameter, and each cell contains an
%array of output values for its corresponding set of parameters. Each
%column of this array is a different output value, and the rows represent
%iteration over the sweep of motion of the mechanism. If a set of
%parameters is invalid, the cell at that location will contain NaN.
optim_data = cell(model_step_count, model_step_count, model_step_count, model_step_count);

%% Run model, collect output data

%Create loading bar dialog
h = waitbar(0,'1','Name','Running model. Please wait...',...
            'CreateCancelBtn',...
            'setappdata(gcbf,''canceling'',1)');
setappdata(h,'canceling',0);
stop_process = 0;
counter = 1;

%Iterate over parameters
for i = 1:length(button_contact_x_i)
    for j = 1:length(button_contact_y)
        for k = 1:length(cam_sweep)
            for l = 1:length(cam_end_dist)
                
                %Organize inputs
                indeps = struct('button_contact_x_i',   button_contact_x_i(i), ...
                                'button_contact_y',     button_contact_y(j), ...
                                'cam_sweep',            cam_sweep(k), ...
                                'cam_end_dist',         cam_end_dist(l) ...
                               );
                
                %Run model
                [cam_moment_ratio, beam_length, button_travel] = ...
                    eject_model(indeps, params, consts);
                
                %Filter based on physical impossibilities
                if any(~all(isreal([cam_moment_ratio, beam_length, button_travel])))
                    optim_data{i,j,k,l} = [NaN, NaN, NaN];
                elseif any(~all(isfinite([cam_moment_ratio, beam_length, button_travel])))
                    optim_data{i,j,k,l} = [NaN, NaN, NaN];
                elseif min(button_travel) < 0
                    optim_data{i,j,k,l} = [NaN, NaN, NaN];
                elseif max(button_travel) > max_button_travel
                    optim_data{i,j,k,l} = [NaN, NaN, NaN];
                elseif indeps.button_contact_y + indeps.cam_end_dist + ...
                        params.cam_end_rad < consts.button_to_lever
                    optim_data{i,j,k,l} = [NaN, NaN, NaN];
                elseif indeps.cam_end_dist + params.cam_end_rad > ...
                        button_to_wall + indeps.button_contact_x_i
                    optim_data{i,j,k,l} = [NaN, NaN, NaN];
                elseif min(beam_length) < 0
                    optim_data{i,j,k,l} = [NaN, NaN, NaN];
                else
                    optim_data{i,j,k,l} = ...
                        [mean(cam_moment_ratio), ...
                         max(beam_length), ...
                         max(button_travel)];
                end
                
                %Cancel calculation if the user requests to
                if getappdata(h,'canceling')
                    stop_process = 1;
                    clear optim_data;
                    delete(h);
                    return;
                end
                
                %Update loading bar once every 1000 iterations
                if rem(counter, 1000) == 0
                    waitbar(counter / iteration_count, h, ...
                        ['Iteration count: ', num2str(counter), '/', num2str(iteration_count)]);
                end
                counter = counter + 1;
            end
        end
    end
end
delete(h);

%Mark data up to date
model_update = 0;

%Note calculation time
time_actual = toc / (60 * 60 * 24);
disp(['Model time: ', datestr(time_actual, 'HH:MM:SS.FFF')]);