% Supplementary code for thesis project: "A Simplified Model of Motor
% Control", to produce Fig 2a and 2b.
% To be used with armModel_Incremental.m and armModel_Instant.m
clc; close all; clear all;

%% Parameters and Variables
n_interp = 5000;

% Range of endpoints tested
x_axis = -0.4:0.02:1.6;
y_axis = -1:0.02:1;

% Starting coordinates
th_1i = -0.785; th_2i = 2.0944; th_3i = 0;
x_i = cos(th_1i) + cos(th_2i+th_1i) + 0.2*cos(th_3i+th_2i+th_1i);
y_i = sin(th_1i) + sin(th_2i+th_1i) + 0.2*sin(th_3i+th_2i+th_1i);

% Import distance_data (or compute distances)
load("distances.mat")

% dists_1_wf = zeros(numel(x_axis), numel(y_axis)); dists_1_nf = dists_1_wf; % wf = with feedback 
% dists_2_wf = dists_1_wf; dists_2_nf = dists_1_wf; % nf = no feedback 
% 
% for i = 1:numel(x_axis)
%     for j = 1:numel(y_axis)
%         [x_wf_temp, y_wf_temp, ~, ~, ~, ~, ~] = armModel_Incremental(x_axis(i), y_axis(j), n_interp, th_1i, th_2i, th_3i, 0, 0);
%         [x_nf_temp, y_nf_temp, ~, ~, ~, ~, ~] = armModel_Instant(x_axis(i), y_axis(j), n_interp, th_1i, th_2i, th_3i, 0);
%         
%         % How close did the model get?
%         dists_1_wf(i, j) = sqrt((x_axis(i) - x_wf_temp(n_interp)).^2 + ((y_axis(j) - y_wf_temp(n_interp)).^2));
%         dists_1_nf(i, j) = sqrt((x_axis(i) - x_nf_temp(n_interp)).^2 + ((y_axis(j) - y_nf_temp(n_interp)).^2));
% 
%         % How long was its travelled path?
%         dists_2_a_wf = sum(sqrt((diff(x_wf_temp)).^2 + (diff(y_wf_temp)).^2));
%         dists_2_b_wf = sqrt(((x_i - x_wf_temp(n_interp)).^2) + ((y_i - y_wf_temp(n_interp)).^2));
%         dists_2_wf(i, j) = dists_2_a_wf/dists_2_b_wf;
%         dists_2_a_nf = sum(sqrt((diff(x_nf_temp)).^2 + (diff(y_nf_temp)).^2));
%         dists_2_b_nf = sqrt(((x_i - x_nf_temp(n_interp)).^2) + ((y_i - y_nf_temp(n_interp)).^2));
%         dists_2_nf(i, j) = dists_2_a_nf/dists_2_b_nf;
%     end
%     disp("Done with x="+num2str(x_axis(i)))
% end
%
% dists_1(:,:,1) = dists_1_nf; dists_1(:,:,2) = dists_1_wf;
% dists_2(:,:,1) = dists_2_nf; dists_2(:,:,2) = dists_1_wf;

%% Plot

% With and Without Feedback
for i = 1:2
    fig = figure('units','normalized','outerposition',[0 0 1 1]);
    %colormap(heatmap_cm);
    col = cividis();
    colormap(col(1:220,:))
    
    % Fig 2a/b-right
    subplot(1, 2, 1)
    imagesc([-0.4, 1.6], [1, -1], rot90(squeeze(-dists_1(:,:,i)))); 
    colorbar; caxis([-1.5 0]);

    % Appearance
    set(gca,'YDir','normal');
    viscircles([x_i, y_i], 0.07, 'Color', (1/256)*[255, 255, 255], 'LineWidth', 5, 'LineStyle', ':', 'EnhanceVisibility', false);
    xticks([-0.4, -0, 0.4, 0.8, 1.2, 1.6]); xticklabels([-0.4, -0, 0.4, 0.8, 1.2, 1.6]);
    yticks([-1, -0.6, -0.2, 0.2, 0.6, 1]); yticklabels([-1, -0.6, -0.2, 0.2, 0.6, 1]);
    ax = gca; ax.FontSize = 14; 
    axis square
    
    % Fig 2a/b-left
    subplot(1, 2, 2)
    imagesc([-0.4, 1.6], [1, -1], rot90(squeeze(dists_2(:,:,i))))
    colorbar; caxis([1 1.5]);

    % Appearance
    set(gca,'YDir','normal');
    viscircles([x_i, y_i], 0.07, 'Color', (1/256)*[255, 255, 255], 'LineWidth', 5, 'LineStyle', ':', 'EnhanceVisibility', false);
    xticks([-0.4, -0, 0.4, 0.8, 1.2, 1.6]); xticklabels([-0.4, -0, 0.4, 0.8, 1.2, 1.6]);
    yticks([-1, -0.6, -0.2, 0.2, 0.6, 1]); yticklabels([-1, -0.6, -0.2, 0.2, 0.6, 1]);
    ax = gca; ax.FontSize = 14; 
    axis square
end