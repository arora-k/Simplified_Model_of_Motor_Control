% This function iteratively computes the trajectory of a 3-DOF arm between given initial and desired positions
function [x, y, th1, th2, th3, lift_drop, open_fold, m] = armModel_Incremental(x_f, y_f, n_interp, th_1i, th_2i, th_3i, cond, delay)
    % Inputs:
    % x_f, y_f - Desired Endpoint Coordinates
    % n_interp - Number of points to interpolate between positions
    % th_1i, th_2i, th_3i - Initial values of shoulder, elbow, and wrist angles respectively (rad)
    % cond - Noise condition (0-15), binary, on/off for four levels
    % delay - desired cortical information delay (in timepoints)
    %
    % Outputs:
    % x, y - (1 x n_interp) arrays of computed endpoint trajectory
    % th1, th2, th3 - (1 x n_interp) arrays of shoulder, elbow and wrist angle values
    % lift_drop, open_fold - (1 x n_interp) arrays of synergy values
    % m - (6 x n_interp) array of muscle activations

    % Initial and final endpoint coordinates
    x_i = cos(th_1i) + cos(th_2i+th_1i) + 0.2*cos(th_3i+th_2i+th_1i);
    y_i = sin(th_1i) + sin(th_2i+th_1i) + 0.2*sin(th_3i+th_2i+th_1i);
    [phi_f, r_f] = cart2pol(x_f, y_f);
    
    % Segment lengths
    l1 = 1; l2 = 1; l3 = 0.2; % upper arm, lower arm, hand

    % Initilalise variables
    th1 = zeros(1,n_interp); th2 = th1; th3 = th1; % angle arrays
    lift_drop = th1; open_fold = th1; % synergy arrays
    m = zeros(6,n_interp); % muscle array
    % Set initial angles
    th1(1) = th_1i; th2(1) = th_2i; th3(1) = th_3i;
    th1(2) = th_1i; th2(2) = th_2i; th3(2) = th_3i;
    
    % Translate noise condition to binary for title label
    noises = zeros(4, 1); % coords, syns, muscles, angles
    noises(:,1) = dec2bin(cond, 4) - '0'; 

    % for each timepoint
    for i = 2:n_interp-1
        % what are the current coordinates (with information delay if selected)
        if i<=delay
            [phi_c, r_c] = cart2pol(x_i, y_i);
        else
            tp = i-delay; % compute coordinates based on old position
            x_c = l1*cos(th1(tp)) + l2*cos(th2(tp)+th1(tp)) + l3*cos(th3(tp)+th2(tp)+th1(tp));
            y_c = l1*sin(th1(tp)) + l2*sin(th2(tp)+th1(tp)) + l3*sin(th3(tp)+th2(tp)+th1(tp));
            [phi_c, r_c] = cart2pol(x_c, y_c);
        end
        
        % distance from final coordinates
        del_phi = phi_f - phi_c + noises(1)*normrnd(0, 0.3*abs(phi_f - phi_c));
        del_r = r_f - r_c + noises(1)*normrnd(0, 0.3*abs(r_f - r_c));
        
        % synergy activations based on distance from final coordinates
        % - set coord-syn coefficients
        scaling = 0.001;
        alpha_1 = 0.5*scaling*1.1; beta_1 = scaling*0.5; % phi range varies about twice as much, so angle coeffs are halved
        alpha_2 = 0.5*scaling*0.1; beta_2 = scaling*1;

        % - activation of synergies
        % -- 1) LIFT-DROP synergy
        lift_drop(i) = alpha_1*(del_phi) - beta_1*(del_r); % alpha > beta, more angle dependent
        % -- 2) OPEN-FOLD synergy
        open_fold(i) = - alpha_2*(del_phi) + beta_2*(del_r); % alpha << beta, mainly dist dependent 
        
        % - add noise
        lift_drop(i) = lift_drop(i) + noises(2)*normrnd(0, 0.3*abs(lift_drop(i)));
        open_fold(i) = open_fold(i) + noises(2)*normrnd(0, 0.3*abs(open_fold(i)));

        % muscle activations based on synergies
        % - set syn-musc coefficients
        scaling_2 = 2000; scaling_3 = 0.001;
        ld_m1 = 2; ld_m3 = 1; ld_m5 = 0.5; ld_m2 = ld_m1; ld_m4 = ld_m3; ld_m6 = ld_m5;
        of_m1 = 0.25; of_m3 = 2; of_m5 = 1; of_m2 = of_m1; of_m4 = of_m3; of_m6 = of_m5;
        
        % - activations of muscles based on synergies
        m1 = (ld_m1*sigmoid(scaling_2*lift_drop(i)) + of_m1*sigmoid(scaling_2*open_fold(i)))*scaling_3; m1 = m1 + noises(3)*normrnd(0, 0.3*abs(m1));
        m2 = (ld_m2*sigmoid(-scaling_2*lift_drop(i)) + of_m2*sigmoid(-scaling_2*open_fold(i)))*scaling_3; m2 = m2 + noises(3)*normrnd(0, 0.3*abs(m2));
        m3 = (ld_m3*sigmoid(scaling_2*lift_drop(i)) + of_m3*sigmoid(-scaling_2*open_fold(i)))*scaling_3; m3 = m3 + noises(3)*normrnd(0, 0.3*abs(m3));
        m4 = (ld_m4*sigmoid(-scaling_2*lift_drop(i)) + of_m4*sigmoid(scaling_2*open_fold(i)))*scaling_3; m4 = m4 + noises(3)*normrnd(0, 0.3*abs(m4));
        m5 = (ld_m5*sigmoid(scaling_2*lift_drop(i)) + of_m5*sigmoid(-scaling_2*open_fold(i)))*scaling_3; m5 = m5 + noises(3)*normrnd(0, 0.3*abs(m5));
        m6 = (ld_m6*sigmoid(-scaling_2*lift_drop(i)) + of_m6*sigmoid(scaling_2*open_fold(i)))*scaling_3; m6 = m6 + noises(3)*normrnd(0, 0.3*abs(m6));
        m(:,i) = [m1; m2; m3; m4; m5; m6]; % muscles x timepoints

        % angle activations based on muscles
        % - set musc-ang coefficiencts
        m1_th1 = 1; m2_th1 = -1; m3_th2 = 1; m4_th2 = -1; m5_th3 = 1; m6_th3 = -1;

        % - angle changes based on muscle activations
        del_th1 = m1_th1*m1 + m2_th1*m2;
        del_th2 = m3_th2*m3 + m4_th2*m4;
        del_th3 = m5_th3*m5 + m6_th3*m6;

        % update current position
        th1(i+1) = th1(i) + del_th1 + noises(4)*normrnd(0, 0.005); 
        th2(i+1) = th2(i) + del_th2 + noises(4)*normrnd(0, 0.005); 
        th3(i+1) = th3(i) + del_th3 + noises(4)*normrnd(0, 0.005);
    end

    % endpoint trajectory
    x = l1*cos(th1) + l2*cos(th2+th1) + l3*cos(th3+th2+th1);
    y = l1*sin(th1) + l2*sin(th2+th1) + l3*sin(th3+th2+th1);
    
    % sigmoid function
    function y = sigmoid(x)
    y = (1/(1 + exp(-x)));
    end
end