%MASTER
% 
% Script to run necessary processes and present resulting data to the user
% 
% Matthew R. Bonanni
% 04-2017

%% Run processes
if ~exist('model_update', 'var')
    run_model;
    optimize;
elseif model_update
    run_model;
    optimize;
elseif optim_update
    optimize;
end

%% Present results

%1-dimensional projection of weighted score data
visrep = reshape(weighted_score,  [numel(weighted_score), 1]);
visrep = visrep(~isnan(visrep));
figure(1);
plot(visrep);

%5-dimensional representation
[x, y, z] = meshgrid(button_contact_x_i, button_contact_y, cam_sweep);
figure(2);
view(3);
title('Weighted Score');
xlabel('Contact Offset');
ylabel('Button Height');
zlabel('Cam Sweep');
c = colorbar;
caxis([min(weighted_score(:)), optimal_score]);
c.Label.String = 'Weighted Score';
hold on;
for i = 1:length(button_contact_x_i)
    score_slice = weighted_score(:,:,:,i);
    plot_data = scatter3(x(:), y(:), z(:), [], score_slice(:), 'filled');
    note = annotation('textbox', [0.03 0.9 0.3 0.05], ...
                      'String', ['Cam Arm Dist: ', num2str(cam_end_dist(i))], ...
                      'FitBoxToText', 'on');
    drawnow;
    animated_data(i) = getframe(gcf);
    pause(10 / length(button_contact_x_i));
    delete(plot_data);
    delete(note);
end

% animation = VideoWriter('pad_eject_optimization', 'MPEG-4');
% animation.FrameRate = 6;
% animation.open;
% animation.writeVideo(animated_data);
% animation.close;
% clear animated_data animation;

%Display relevant mechanism data
disp([char(10), 'Input Parameters:']);
disp(['Button contact offset: ', num2str(optim_button_contact_x_i)]);
disp(['Button height: ', num2str(optim_button_contact_y)]);
disp(['Cam sweep angle: ', num2str(optim_cam_sweep)]);
disp(['Cam arm distance: ', num2str(optim_cam_end_dist)]);

disp([char(10), 'Output Data:']);
disp(['Cam moment ratio: ', num2str(cam_moment_ratio(i_max, j_max, k_max, l_max))]);
disp(['Beam length: ', num2str(beam_length(i_max, j_max, k_max, l_max))]);
disp(['Button travel: ', num2str(button_travel(i_max, j_max, k_max, l_max))]);