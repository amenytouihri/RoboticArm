%% workspace plot - we need to add in constraint of arms hitting each other
% (alpha 1 or 2 can't be ~0 or ~180)

clear; clc; close all;

% measurements
l0 = 23.144;
l1 = 76; %61
l2 = 76; %93.6
l4 = 50; %50.91799985

% defined grid of (x,y) points to test
x_min = -40; % -80
x_max =  40; % 80
y_min =   -l4; %0
y_max =  80 + l4;   % 80 + l4

Nx = 100;   % number of points in x
Ny = 100;   % number of points in y

[x_grid, y_grid] = meshgrid( linspace(x_min, x_max, Nx), ...
                             linspace(y_min, y_max, Ny) );

reachable_mask = false(size(x_grid));  % same size as grid

% check reachability for each point in grid using ik
for i = 1:Ny
    for j = 1:Nx
        x = x_grid(i,j);
        y = y_grid(i,j);

        [theta1_deg, theta2_deg] = calculate_ik_5bar(x, y, l0, l1, l2, l4);

        if ~isnan(theta1_deg) && ~isnan(theta2_deg)
            reachable_mask(i,j) = true;
        end
    end
end

% extract reachable points
x_reach = x_grid(reachable_mask);
y_reach = y_grid(reachable_mask);

% plot the workspace
figure; hold on; axis equal;
scatter(x_reach, y_reach, 5, 'b', '.');  % reachable points

xlabel('X (mm)');
ylabel('Y (mm)');
title('Reachable Workspace of 5-bar Robot (Sampled via IK)');
grid on;

% draws base locations of the two actuated joints (helps visualise offset)
plot(-l0, -l4, 'ro', 'MarkerFaceColor', 'r');   % left base
plot( l0, -l4, 'ro', 'MarkerFaceColor', 'r');   % right base
legend('Reachable points', 'Left base', 'Right base', 'Location', 'best');


function [theta1_deg, theta2_deg] = calculate_ik_5bar(x, y, l0, l1, l2, l4)
% uses below function, same ik as in python
% returns motor angles in degrees.
% if point unreachable, returns NaN, NaN.

    %%% NEW: alpha limits in radians (16°–175°) %%%
    alpha_min = deg2rad(16);
    alpha_max = deg2rad(175);

    % workspace check (here you're allowing a big box)
    % if ~( -40 <= x && x <= 40 && 0 <= y && y <= 80 + l4 )
    if ~( -200 <= x && x <= 200 && -l4 <= y && y <= 200 + l4 )
        theta1_deg = NaN;
        theta2_deg = NaN;
        return;
    end

    try
        % left arm: base at (-l0, -l4) 
        x_vec1 = x + l0;
        y_vec1 = y + l4;
        D1_sq = x_vec1^2 + y_vec1^2;
        D1    = sqrt(D1_sq);

        term1 = (l1^2 + D1_sq - l2^2) / (2 * l1 * D1);
        if ~( -1.0 <= term1 && term1 <= 1.0 )
            theta1_deg = NaN; theta2_deg = NaN; return;
        end

        alpha_1 = acos(term1);           % in radians
        beta_1  = atan2(y_vec1, x_vec1);

        %%% NEW: reject if alpha_1 outside [16°, 175°] %%%
        if alpha_1 < alpha_min || alpha_1 > alpha_max
            theta1_deg = NaN; theta2_deg = NaN; return;
        end

        theta_1a = beta_1 + alpha_1;
        theta_1b = beta_1 - alpha_1;

        % right arm: base at ( l0, -l4)
        x_vec2 = x - l0;
        y_vec2 = y + l4;
        D2_sq  = x_vec2^2 + y_vec2^2;
        D2     = sqrt(D2_sq);

        term2 = (l1^2 + D2_sq - l2^2) / (2 * l1 * D2);
        if ~( -1.0 <= term2 && term2 <= 1.0 )
            theta1_deg = NaN; theta2_deg = NaN; return;
        end

        alpha_2 = acos(term2);          % in radians
        beta_2  = atan2(y_vec2, x_vec2);

        %%% NEW: reject if alpha_2 outside [16°, 175°] %%%
        if alpha_2 < alpha_min || alpha_2 > alpha_max
            theta1_deg = NaN; theta2_deg = NaN; return;
        end
        
        theta_2a = beta_2 + alpha_2;
        theta_2b = beta_2 - alpha_2;

        % two possible solution pairs (in radians)
        sols = [theta_1a, theta_2a;
                theta_1b, theta_2b];

        % choose pair closest to (0,0)
        best_idx = 1;
        min_mag  = inf;
        for k = 1:2
            t1 = sols(k,1);
            t2 = sols(k,2);
            mag = sqrt(t1^2 + t2^2);
            if mag < min_mag
                min_mag  = mag;
                best_idx = k;
            end
        end

        best_t1 = sols(best_idx,1);
        best_t2 = sols(best_idx,2);

        theta1_deg = rad2deg(best_t1);
        theta2_deg = rad2deg(best_t2);

    catch
        theta1_deg = NaN;
        theta2_deg = NaN;
    end
end
