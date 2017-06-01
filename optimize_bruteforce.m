%OPTIMIZE_BRUTEFORCE
% 
% Script to analyze brute force model output data and determine the optimal
% input parameters. The following 3 data points are optimized:
% 
%   cam_moment_ratio        Cam input/output moment ratio   (maximized)
%   beam_length             Length of cam beam              (minimized)
%   button_travel           Maximum button depression       (minimized)
% 
% In order to determine the optimal set of input parameters, these 3 points
% must be assigned a relative weight. In this way, the output data can be
% normalized and weighted, and a score can be assigned to each set of input
% parameters. The maximum score corresponds to the optimal parameters.
% 
% Matthew R. Bonanni
% 04-2017

%% Configuration

%Weights
cam_moment_ratio_weight = 1;
beam_length_weight = -10;
button_travel_weight = -1;

%Mark data out of date
optim_update = 1;

%% Normalize data based on statistical analysis

tic;

% Get specific outputs from optimization data
cam_moment_ratio =  cellfun(@(get_value) get_value(1), optim_data);
beam_length =       cellfun(@(get_value) get_value(2), optim_data);
button_travel =     cellfun(@(get_value) get_value(3), optim_data);

% Flatten into sorted vectors for statistical analysis
cam_moment_ratio_flat = sort(cam_moment_ratio(:));
beam_length_flat =      sort(beam_length(:));
button_travel_flat =    sort(button_travel(:));

%Filter out NaN values
cam_moment_ratio_flat = cam_moment_ratio_flat(~isnan(cam_moment_ratio_flat));
beam_length_flat =      beam_length_flat(~isnan(beam_length_flat));
button_travel_flat =    button_travel_flat(~isnan(button_travel_flat));

%Calculate medians
cam_moment_ratio_median =  median(cam_moment_ratio_flat);
beam_length_median =       median(beam_length_flat);
button_travel_median =     median(button_travel_flat);

%Calculate median absolute deviation of each output
consistency_constant = 1.4826;
cam_moment_ratio_mad =  consistency_constant * mad(cam_moment_ratio_flat, 1);
beam_length_mad =       consistency_constant * mad(beam_length_flat, 1);
button_travel_mad =     consistency_constant * mad(button_travel_flat, 1);

%Determine MAD distance of each point
cam_moment_ratio_mad_dist = abs(cam_moment_ratio - cam_moment_ratio_median) / cam_moment_ratio_mad;
beam_length_mad_dist = abs(beam_length - beam_length_median) / beam_length_mad;
button_travel_mad_dist = abs(button_travel - button_travel_median) / button_travel_mad;

%Remove outliers
mad_cutoff = 2;
cam_moment_ratio_outliers = cam_moment_ratio_mad_dist > mad_cutoff;
beam_length_outliers = beam_length_mad_dist > mad_cutoff;
button_travel_outliers = button_travel_mad_dist > mad_cutoff;

outliers = cam_moment_ratio_outliers + beam_length_outliers + button_travel_outliers;
outliers(outliers > 1) = 1;
outliers = logical(outliers);

cam_moment_ratio(outliers) = NaN;
beam_length(outliers) = NaN;
button_travel(outliers) = NaN;

%Determine range of each output data set
cam_moment_ratio_range =    [min(cam_moment_ratio(:)),    max(cam_moment_ratio(:))];
beam_length_range =         [min(beam_length(:)),         max(beam_length(:))];
button_travel_range =       [min(button_travel(:)),       max(button_travel(:))];

%Normalize data sets
cam_moment_ratio_norm = (cam_moment_ratio - cam_moment_ratio_range(1)) / ...
    (cam_moment_ratio_range(2) - cam_moment_ratio_range(1));
beam_length_norm = (beam_length - beam_length_range(1)) / ...
    (beam_length_range(2) - beam_length_range(1));
button_travel_norm = (button_travel - button_travel_range(1)) / ...
    (button_travel_range(2) - button_travel_range(1));

%% Score data and determine optimal parameters

%Tally score
weighted_score = cam_moment_ratio_norm *    cam_moment_ratio_weight + ...
                 beam_length_norm *         beam_length_weight + ...
                 button_travel_norm *       button_travel_weight;

%Find optimal parameters
[optimal_score, position] = max(weighted_score(:));
[i_max, j_max, k_max, l_max] = ind2sub(size(weighted_score), position);

optim_button_contact_x_i = button_contact_x_i(i_max);
optim_button_contact_y = button_contact_y(j_max);
optim_cam_sweep = cam_sweep(k_max);
optim_cam_end_dist = cam_end_dist(l_max);

%Mark data up to date
optim_update = 0;

%Note calculation time
optim_time = toc;
optim_time_days = optim_time / (60 * 60 * 24);
disp(['Optimization time: ', datestr(optim_time_days, 'HH:MM:SS.FFF')]);