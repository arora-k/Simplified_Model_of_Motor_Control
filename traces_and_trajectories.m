% Supplementary code for thesis project: "A Simplified Model of Motor
% Control", to show traces of model units during a given movement.
% To be used with armModel_Incremental.m and armModel_Instant.m
close all; clear all; clc;

%% Parameters
l1 = 1; l2 = 1; l3 = 0.2; % arm segment lengths
n_interp = 5000; duration = 0.2; % sec

% Initial and Final points data
num_trajs = 4;   
theta_i = [-0.785, 2.0944, 0; 0.7693, 2.7791, 0]; % shoudler, elbow, wrist
pos = [1.3, 1.3; 0.2, 0.1]; % x, y

% Label strings
switch_labels = containers.Map([1, 0], ["Yes", "No"]);
level_labels = ["B: ", " S: ", " M: ", " A: "];

%% Run model and Plot Trajectories/Traces

for noise_cond = 1:1% 
    for tr = 1:4
        % Set angles and positions
        ths = theta_i(floor((tr-1)/2)+1, :);
        th_1i = ths(1); th_2i = ths(2); th_3i = ths(3);
        x_f = pos(mod(tr,2)+1,1); y_f = pos(mod(tr,2)+1,2);

        x_i = cos(th_1i) + cos(th_2i+th_1i) + 0.2*cos(th_3i+th_2i+th_1i);
        y_i = sin(th_1i) + sin(th_2i+th_1i) + 0.2*sin(th_3i+th_2i+th_1i);
        [phi_f, r_f] = cart2pol(x_f, y_f);

        % Compute Trajectory
        [x, y, th1, th2, th3, syn1, syn2, m] = armModel_Incremental(x_f, y_f, n_interp, th_1i, th_2i, th_3i, noise_cond-1, 50);
        %[x, y, th1, th2, th3, syn1, syn2, m] = armModel_Instant(x_f, y_f, n_interp, th_1i, th_2i, th_3i, noise_cond-1);
        [phi, r] = cart2pol(x, y);
        % Compute distances from desired endpoint
        dists_phi = phi_f - phi; dists_r = r_f - r;

        % Plot parameters
        fontSize = 16; fontSizeMini = 14;
        t_axis = linspace(0, duration, n_interp);
        n_arm = [1, floor(0.2*n_interp), n_interp];
        arm_cols = (1/256)*[[199, 216, 237]; [152, 171, 214]; [82, 106, 161]]; arm_widths = [0.5, 1, 2];
        
        % Noise condition
        switches = dec2bin(noise_cond-1, 4) - '0';
        noise_labels = "";
        for i = 1:numel(switches)
            noise_labels = noise_labels.append(level_labels(i) + switch_labels(switches(i)));
        end
        titleName = "Arm trajectory (Cortical FB, Noise- "+noise_labels+")";

        % 1) Trajectory plot
        fig = figure('units','normalized','outerposition',[0 0 1 1]);
        disp(num2str(noise_cond-1)+" "+num2str(tr))
        subplot(4, 4, [1, 2, 5, 6, 9, 10, 13, 14])
        plot(x,y, 'k--', 'Linewidth', 3)
        hold on

        % For each selected timepoint
        for i = 1:numel(n_arm)
            % plot the three segments
            line([0, l1*cos(th1(n_arm(i)))],[0, l1*sin(th1(n_arm(i)))], 'Color', arm_cols(i,:), 'LineWidth', arm_widths(i))
            line([l1*cos(th1(n_arm(i))), l1*cos(th1(n_arm(i))) + l2*cos(th1(n_arm(i))+th2(n_arm(i)))], ...
                [l1*sin(th1(n_arm(i))), l1*sin(th1(n_arm(i))) + l2*sin(th1(n_arm(i))+th2(n_arm(i)))], 'Color', arm_cols(i,:), 'LineWidth', arm_widths(i))
            line([l1*cos(th1(n_arm(i))) + l2*cos(th1(n_arm(i))+th2(n_arm(i))), l1*cos(th1(n_arm(i))) + l2*cos(th1(n_arm(i))+th2(n_arm(i))) + l3*cos(th1(n_arm(i))+th2(n_arm(i))+th3(n_arm(i)))], ...
                [l1*sin(th1(n_arm(i))) + l2*sin(th1(n_arm(i))+th2(n_arm(i))), l1*sin(th1(n_arm(i))) + l2*sin(th1(n_arm(i))+th2(n_arm(i))) + l3*sin(th1(n_arm(i))+th2(n_arm(i))+th3(n_arm(i)))], 'Color', arm_cols(i,:), 'LineWidth', arm_widths(i))
        end
        
        % Appearance
        xlim([-0.1, 1.9]); ylim([-1.25, 1.75]);
        legend('Endpoint Trajectory', 'Start', '20% Through', 'End', 'Location', 'Northwest')
        xlabel('x-position', 'fontsize', fontSize); ylabel('y-position', 'fontsize', fontSize); 
        title(titleName, 'fontsize', fontSize)
        viscircles([0, 0], 0.01, 'Color', 'k', 'LineWidth', 3);
        viscircles([x_f, y_f], 0.05, 'Color', [0.6350 0.0780 0.1840], 'LineWidth', 1);
        daspect([1 1 1 ])

        % 2) Polar Coordinate Deviations
        subplot(4, 4, [3, 4])
        yyaxis left
        plot(t_axis, dists_phi, 'LineWidth', 1.5); hold on;
        ylim([-3, 3]); ylabel('$\Delta \phi$','Interpreter','latex', 'fontsize', fontSizeMini)

        yyaxis right
        plot(t_axis, dists_r, 'LineWidth', 1.5);
        ylim([-1.5, 1.5]); ylabel('$\Delta r$','Interpreter','latex', 'fontsize', fontSizeMini)

        yline(0, 'HandleVisibility', 'off');
        title('Distance from desired polar coordinates', 'fontsize', fontSizeMini)

        % 3) Behavioural Unit Traces
        subplot(4, 4, [7, 8])
        plot(t_axis, syn1, 'LineWidth', 1.5); hold on;
        plot(t_axis, syn2, 'LineWidth', 1.5)
        legend('Lift-Drop', 'Open-Fold', 'fontsize', fontSizeMini-3);
        title('Behaviour Activations');
        yline(0, 'HandleVisibility', 'off');

        % 4) Muscle Unit Traces
        subplot(4, 4, [11, 12])
        hold on;
        plot(t_axis, m(1,:), 'LineWidth', 1.5); plot(t_axis, m(2,:), 'LineWidth', 1.5);
        plot(t_axis, m(3,:), 'LineWidth', 1.5); hold on; plot(t_axis, m(4,:), 'LineWidth', 1.5); hold on;
        plot(t_axis, m(5,:), 'LineWidth', 1.5); hold on; plot(t_axis, m(6,:), 'LineWidth', 1.5); hold on;
        legend('M1', 'M2', 'M3', 'M4', 'M5', 'M6', 'fontsize', fontSizeMini-3);
        title('Muscle Activations');
        yline(0, 'HandleVisibility', 'off');

        % 5) Angular Velocities of Joints
        subplot(4, 4, [15, 16])
        plot(t_axis(1:n_interp-1), diff(th1)./(duration/n_interp), 'LineWidth', 1.5)
        hold on
        plot(t_axis(1:n_interp-1), diff(th2)./(duration/n_interp), 'LineWidth', 1.5)
        plot(t_axis(1:n_interp-1), diff(th3)./(duration/n_interp), 'LineWidth', 1.5)
        legend('Shoulder', 'Elbow', 'Wrist', 'fontsize', fontSizeMini-3)
        title('Angular Velocities', 'fontsize', fontSizeMini); 
        xlabel('Time (s)', 'fontsize', fontSizeMini); ylabel('rad/s', 'fontsize', fontSizeMini)
        yline(0, 'HandleVisibility', 'off');

        % save figure
        a = num2str(round(x_i, 2)); b = num2str(round(y_i, 2));
        c = num2str(round(x_f, 2)); d = num2str(round(y_f, 2));
        %saveas(fig, "3_V0_N"+num2str(noise_cond-1)+"_["+a+", "+b+"] - ["+c+", "+d+"].png")
    end
end
