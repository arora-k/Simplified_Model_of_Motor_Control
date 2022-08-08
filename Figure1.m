% Supplementary code for thesis project: "A Simplified Model of Motor
% Control", to produce Fig 1a and 1b.
% To be used with armModel_Incremental.m and armModel_Instant.m
close all; clear all; clc;

%% Parameters
l1 = 1; l2 = 1; l3 = 0.2; % arm segment lengths
n_interp = 5000; duration = 0.2; % sec
noise_cond = 0; delay = 0;
% Fig 1a
theta_i = [0.7693, 2.7791, 0]; % shoudler, elbow, wrist
x_f = 1.3; y_f = 1.3; % x, y
% % Fig 1b
% theta_i = [-0.785, 2.0944, 0]; % shoudler, elbow, wrist
% x_f = 1.3; y_f = 1.3; % x, y

%% Run Models
% Set Angles and Positions
x_i = cos(theta_i(1)) + cos(theta_i(2)+theta_i(1)) + 0.2*cos(theta_i(3)+theta_i(2)+theta_i(1));
y_i = sin(theta_i(1)) + sin(theta_i(2)+theta_i(1)) + 0.2*sin(theta_i(3)+theta_i(2)+theta_i(1));
[phi_f, r_f] = cart2pol(x_f, y_f);

% Compute Trajectory
[x_wf, y_wf, th1_wf, th2_wf, th3_wf, ~, ~, ~] = armModel_Incremental(x_f, y_f, n_interp, theta_i(1), theta_i(2), theta_i(3), noise_cond, 50);
[x_nf, y_nf, th1_nf, th2_nf, th3_nf, ~, ~, ~] = armModel_Instant(x_f, y_f, n_interp, theta_i(1), theta_i(2), theta_i(3), noise_cond);

%% Plot

% Parameters
fontSize = 16;
fontSizeMini = 14;
fontSizeMicro = 10;
arm_cols = (1/256)*[[133, 131, 131]; [176, 23, 23]; [82, 106, 161]]; 
arm_widths = [0.5, 1, 2];

% Trajectories
fig = figure('units','normalized','outerposition',[0 0 1 1]);
hold on
plot(x_wf, y_wf, 'b--', 'Linewidth', 3)
plot(x_nf, y_nf, 'r--', 'Linewidth', 3)
viscircles([x_f, y_f], 0.05, 'Color', (1/256)*[18, 17, 17], 'LineWidth', 4);

% Initial arm
plotArm(1, arm_cols(1,:), arm_widths(1), l1, l2, l3, th1_wf, th2_wf, th3_wf)

% Final arms
plotArm(n_interp, arm_cols(3,:), arm_widths(3), l1, l2, l3, th1_wf, th2_wf, th3_wf)
plotArm(n_interp, arm_cols(2,:), arm_widths(3), l1, l2, l3, th1_nf, th2_nf, th3_nf)
axis equal

% Formatting, Appearance
legend('With Feedback Trajectory', 'No Feedback Trajectory', 'Desired Endpoint', 'Initial Arm', ...
    'Location', 'Northwest', 'FontSize', fontSizeMini);
viscircles([0, 0], 0.01, 'Color', 'k', 'LineWidth', 3);
ax = gca; ax.FontSize = fontSizeMicro+2; 
box on;

xlim([-0.5, 1.5]); ylim([-0.75, 1.75]); %1a
%xlim([-0.15, 1.85]); ylim([-1, 1.5]); %1b

% ---------------

function plotArm(t, col, thickness, l1, l2, l3, th1, th2, th3)
    line([0, l1*cos(th1(t))],[0, l1*sin(th1(t))], 'Color', col, 'LineWidth', thickness)
    line([l1*cos(th1(t)), l1*cos(th1(t)) + l2*cos(th1(t)+th2(t))], ...
        [l1*sin(th1(t)), l1*sin(th1(t)) + l2*sin(th1(t)+th2(t))], 'Color', col, 'LineWidth', thickness)
    line([l1*cos(th1(t)) + l2*cos(th1(t)+th2(t)), l1*cos(th1(t)) + l2*cos(th1(t)+th2(t)) + l3*cos(th1(t)+th2(t)+th3(t))], ...
        [l1*sin(th1(t)) + l2*sin(th1(t)+th2(t)), l1*sin(th1(t)) + l2*sin(th1(t)+th2(t)) + l3*sin(th1(t)+th2(t)+th3(t))], 'Color', col, 'LineWidth', thickness)
end