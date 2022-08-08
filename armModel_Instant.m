% This function computes the end configuration of a 3-DOF arm across given initial and desired positions
function [x, y, th1, th2, th3, lift_drop, open_fold, m] = armModel_Instant(x_f, y_f, n_interp, th_1i, th_2i, th_3i, cond)
    % Inputs:
    % x_f, y_f - Desired Endpoint Coordinates
    % n_interp - Number of points to interpolate between positions
    % th_1i, th_2i, th_3i - Initial values of shoulder, elbow, and wrist angles respectively (rad)
    % cond - Noise condition (0-15), binary, on/off for four levels
    %
    % Outputs:
    % x, y - (1 x n_interp) arrays of computed endpoint trajectory
    % th1, th2, th3 - (1 x n_interp) shoulder, elbow and wrist angle values
    % lift_drop, open_fold - (1 x 1) synergy values
    % m - (6 x 1) muscle activations
    
    % Desired end coordinates
    [phi_f, r_f] = cart2pol(x_f, y_f);
    l1 = 1; l2 = 1; l3 = 0.2;

    % variables
    th1 = zeros(1,n_interp); th2 = th1; th3 = th1;
    th1(1) = th_1i; th2(1) = th_2i; th3(1) = th_3i;
    noises = zeros(4, 1); % coords, syns, muscles, angles
    noises(:,1) = dec2bin(cond, 4) - '0'; 
    
    % current position coordinates
    x_i = l1*cos(th1(1)) + l2*cos(th2(1)+th1(1)) + l3*cos(th3(1)+th2(1)+th1(1));
    y_i = l1*sin(th1(1)) + l2*sin(th2(1)+th1(1)) + l3*sin(th3(1)+th2(1)+th1(1));
    dist = sqrt((x_f-x_i)^2 + (y_f-y_i)^2);
    [phi_c, r_c] = cart2pol(x_i, y_i);
    phi_c = phi_c + noises(1)*normrnd(0, 0.1*abs(phi_f - phi_c));
    r_c = r_c + noises(1)*normrnd(0, 0.1*abs(r_f - r_c));
    
    % synergy activations based on distance from final coordinates
    % - set coord-syn coefficients
    scaling = 1;
    alpha_1 = 0.5*scaling*1.1; beta_1 = scaling*0.5; % phi range varies about twice as much, so angle coeffs are halved
    alpha_2 = 0.5*scaling*0.1; beta_2 = scaling*1; 

    % - activation of synergies
    % -- 1) LIFT-DROP synergy
    lift_drop = alpha_1*(phi_f - phi_c) - beta_1*(r_f - r_c); % alpha > beta, more angle dependent
    % -- 2) OPEN-FOLD synergy
    open_fold = - alpha_2*(phi_f - phi_c) + beta_2*(r_f - r_c); % alpha << beta, mainly dist dependent
    
    lift_drop = lift_drop + noises(2)*normrnd(0, 0.1*abs(lift_drop));
    open_fold = open_fold + noises(2)*normrnd(0, 0.1*abs(open_fold));

    % muscle activations based on synergies
    % - set syn-musc coefficients
    ld_m1 = 2; ld_m3 = 1; ld_m5 = 0.5; ld_m2 = ld_m1; ld_m4 = ld_m3; ld_m6 = ld_m5;
    of_m1 = 0.25; of_m3 = 2; of_m5 = 1; of_m2 = of_m1; of_m4 = of_m3; of_m6 = of_m5;

    % - activations of muscles based on synergies
    scaling_2 = 2; scaling_3 = 0.25*dist;
    m1 = (ld_m1*sigmoid(scaling_2*lift_drop) + of_m1*sigmoid(scaling_2*open_fold))*scaling_3; m1 = m1 + noises(3)*normrnd(0, 0.3*abs(m1));
    m2 = (ld_m2*sigmoid(-scaling_2*lift_drop) + of_m2*sigmoid(-scaling_2*open_fold))*scaling_3; m2 = m2 + noises(3)*normrnd(0, 0.3*abs(m2));
    m3 = (ld_m3*sigmoid(scaling_2*lift_drop) + of_m3*sigmoid(-scaling_2*open_fold))*scaling_3; m3 = m3 + noises(3)*normrnd(0, 0.3*abs(m3));
    m4 = (ld_m4*sigmoid(-scaling_2*lift_drop) + of_m4*sigmoid(scaling_2*open_fold))*scaling_3; m4 = m4 + noises(3)*normrnd(0, 0.3*abs(m4));
    m5 = (ld_m5*sigmoid(scaling_2*lift_drop) + of_m5*sigmoid(-scaling_2*open_fold))*scaling_3; m5 = m5 + noises(3)*normrnd(0, 0.3*abs(m5));
    m6 = (ld_m6*sigmoid(-scaling_2*lift_drop) + of_m6*sigmoid(scaling_2*open_fold))*scaling_3; m6 = m6 + noises(3)*normrnd(0, 0.3*abs(m6));  
    m = [m1; m2; m3; m4; m5; m6]; % muscles x timepoints

    % angle activations based on muscles
    % - set musc-ang coefficiencts
    m1_th1 = 1; m2_th1 = -1; m3_th2 = 1; m4_th2 = -1; m5_th3 = 1; m6_th3 = -1; 

    % - angle changes based on muscle activations
    th_1f = th_1i + m1_th1*m1 + m2_th1*m2 + noises(4)*normrnd(0, 0.5);
    th_2f = th_2i + m3_th2*m3 + m4_th2*m4 + noises(4)*normrnd(0, 0.5);
    th_3f = th_3i + m5_th3*m5 + m6_th3*m6 + noises(4)*normrnd(0, 0.5);

    % endpoint trajectory
    th1 = linspace(th_1i, th_1f, n_interp);
    th2 = linspace(th_2i, th_2f, n_interp);
    th3 = linspace(th_3i, th_3f, n_interp);
    x = l1*cos(th1) + l2*cos(th2+th1) + l3*cos(th3+th2+th1);
    y = l1*sin(th1) + l2*sin(th2+th1) + l3*sin(th3+th2+th1);
    
    % sigmoid function
    function y = sigmoid(x)
    y = (1/(1 + exp(-x)));
    end
end