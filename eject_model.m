function objectives = eject_model(indeps, params, consts)
%EJECT_MODEL Calculate eject mechanism geometry for analysis
%   
%   Takes the following arguments:
%   
%       indeps  Mechanism independent variables. Struct with fields:
%           button_contact_x_i      Button contact initial offset
%           button_contact_y        Button height
%           cam_sweep               Angle swept by cam
%           cam_end_dist            Distance from cam pivot to beam center
%       
%       params  Mechanism semi-permanent parameters. Struct with fields:
%           cam_end_rad             Beam radius
%           lever_end_rad           Arm tip radius
%       
%       consts  Mechanism permanent physical constants. Struct with fields:
%           lever_sweep             Angle swept by lever
%           hinge_x                 x distance from cam/button to lever
%           button_to_lever         y distance from button to lever
%   
%   Uses geometric model (documented in OneNote) to calculate mechanism
%   geometry, including some output parameters to be optimized.
%   
%   objectives = EJECT_MODEL(indeps, params, consts) returns a struct of 
%       the following physical measurements to be optimized:
%       
%       cam_moment_ratio        Cam input/output moment ratio
%       beam_length             Length of cam beam
%       button_travel           Maximum button depression
%   
%   Matthew R. Bonanni
%   04-2017

%% Configuration
cam_step_count = 50;

objectives = struct('cam_moment_ratio',     [], ...
                    'beam_length',          [], ...
                    'button_travel',        []  ...
                    );

%% Create part objects
cam = linkpart;
lever = linkpart;
button_contact = linkpart;
lever_contact = linkpart;

cam.addprop('profile');
cam.addprop('beam');
cam.beam = struct('len',        [], ...
                  'theta',      [], ...
                  'theta_i',    [], ...
                  'theta_f',    [] ...
                 );
cam.addprop('moment_arms');
cam.moment_arms = struct('button_dist', [], ...
                         'lever_dist',  [] ...
                        );

%% Fill structures with input data
hinge_dist = [];

button_contact.x_i = indeps.button_contact_x_i;
button_contact.y = indeps.button_contact_y;
cam.sweep = indeps.cam_sweep;
cam.end_dist = indeps.cam_end_dist;

cam.end_rad = params.cam_end_rad;
lever.end_rad = params.lever_end_rad;

lever.sweep = consts.lever_sweep;
hinge_dist(1) = consts.hinge_x;

%% Calculate mechanism geometry
hinge_dist(2) = consts.button_to_lever - indeps.button_contact_y;

lever_contact.y_f = (hinge_dist(2) - lever.end_rad) * -1;

%Get sweep angles of cam
cam.theta = linspace(0, cam.sweep, cam_step_count)';

%Get sweep angles of beam
cam.y_f = lever_contact.y_f + cam.end_rad;
cam.beam.theta_f = asind(cam.y_f / cam.end_dist);
cam.beam.theta_i = cam.beam.theta_f + cam.sweep;
cam.beam.theta = cam.beam.theta_i - cam.theta;

%Calculate cam profile
cam_data = cam_surf(cam.theta, button_contact.y, button_contact.x_i);

cam.profile = cam_data.profile;
button_contact.x = cam_data.button_travel;

%Calculate contact distances
cam.moment_arms.lever_dist = sqrt(cam.end_dist^2 + cam.end_rad^2 - 2*cam.end_dist*cam.end_rad*cosd(90 - cam.beam.theta));
cam.moment_arms.button_dist = sqrt(button_contact.x .^2 + button_contact.y ^2);
lever_contact.x = cam.end_dist * cosd(cam.beam.theta);

%Calculate ratio of cam moments
objectives.cam_moment_ratio = button_contact.y ./ lever_contact.x;

%Calculate lever sweep angles
cam.y = cam.end_dist .* sind(cam.beam.theta);
lever_contact.y = cam.y - cam.end_rad;
lever.y = lever_contact.y - lever.end_rad;

%NOTE- This section calculated in lever-axis based coordinate system
lever.y = lever.y + hinge_dist(2);
lever.end_dist = lever.y(1) / sind(lever.sweep);
lever.theta = asind(lever.y / lever.end_dist);
lever.x = lever.end_dist * cosd(lever.theta) * -1;
%NOTE- Return to cam coordinate system

lever.x = hinge_dist(1) + lever.x;
cam.beam.len = lever.x;

%For output
objectives.beam_length = cam.beam.len;
objectives.button_travel = button_contact.x;