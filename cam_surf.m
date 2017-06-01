function [cam_data] = cam_surf(cam_theta, h, w0)
%CAM_SURF Analyze cam surface geometry for given physical parameters
%   
%   Takes the following arguments:
%   
%       cam_theta   Column vector of sweep angle values
%       h           Height of button axis from rotation point
%       w0          Offset distance of initial contact point from rotation
%                   point
%   
%   cam_data = CAM_SURF(cam_sweep, h, w0) returns a struct with the
%   following fields:
%       
%       profile     [x, y] cam surface coordinate data
%       button_travel   Vector of button travel values throughout sweep
%   
%   Matthew R. Bonanni
%   04-2017

dtheta = cam_theta(end) / length(cam_theta);

%Set up data structures
button_travel = zeros(length(cam_theta),1);
x = zeros(length(cam_theta),1);
y = zeros(length(cam_theta),1);
button_travel(1) = w0;
x(1) = w0;
y(1) = h;

%Iterative differential equation solve
for i = 2:length(cam_theta)
    
    a = h * tand(dtheta / 2);
    ds = (a + button_travel(i-1)) * tand(dtheta);
    
    dw = (a + button_travel(i-1))/cosd(dtheta) + a - button_travel(i-1);
    dx = ds * sind(cam_theta(i-1)) * -1;
    dy = ds * cosd(cam_theta(i-1));
    
    button_travel(i) = button_travel(i-1) + dw;
    x(i) = x(i-1) + dx;
    y(i) = y(i-1) + dy;
    
end

profile = [x, y];

%Get max travel
[max_travel, i_max_travel] = max(button_travel);

cam_data = struct('profile', profile, ...
                  'button_travel', button_travel);

if nargout == 0
    
    %Plot cam curve
    figure(1);
    hold on
    plot(profile(:,1), profile(:,2), 'b', 'LineWidth', 2);
    line([w0, w0], [-4, 10]);
    line([-5, 10], [h, h]);
    
    %Plot rotated cam curve
    for i = 5:5:cam_theta(end)
        rotation_matrix = [cosd(-i), -sind(-i); sind(-i), cosd(-i)];
        profile_rotated = (rotation_matrix * profile')';
        plot(profile_rotated(:,1), profile_rotated(:,2), 'b', 'LineWidth', 2);
    end
    
    %Cam hinge circles
    plot(0,0,'rx');
    viscircles([0,0],1,'Color', 'b', 'LineStyle', '--');
    viscircles([0,0],3,'Color', 'b', 'LineStyle', '--');
    axis equal;
    title('Cam Face Profile');
    
    %Travel curve
    figure(2);
    hold on
    plot(cam_theta, button_travel);
    title('Travel at Given Angle');
    xlabel('Cam Rotation (deg)');
    ylabel('Button Travel Past Center (mm)');
    plot(cam_theta(i_max_travel), max_travel, 'rx');
    text(cam_theta(end)*0.2, max_travel*0.8, ['Max Travel: ', num2str(max_travel,3), ' mm']);
    
end