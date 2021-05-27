function [total_data_after_kalman] = kalman_estimate(actual_positions,acc)
    %Load actual position data
    CM_idx = actual_positions; %Make sure this order is retained
    [required_runs,cols] = size(CM_idx);
    
    %% define main variables
    dt = 1;  %our sampling rate
    S_frame = 1; %starting frame
    %u = .005; % define acceleration magnitude
    u = acc;
    Q= [CM_idx(S_frame,1); CM_idx(S_frame,2); 0; 0]; %initized state--it has four components: [positionX; positionY; velocityX; velocityY] of the hexbug
    Q_estimate = Q;  %estimate of initial location estimation of where the hexbug is (what we are updating)
    HexAccel_noise_mag = .1; %process noise: the variability in how fast the Hexbug is speeding up (stdv of acceleration: meters/sec^2)
    tkn_x = 1;  %measurement noise in the horizontal direction (x axis).
    tkn_y = 1;  %measurement noise in the horizontal direction (y axis).
    Ez = [tkn_x 0; 0 tkn_y];
    Ex = [dt^4/4 0 dt^3/2 0; ...
        0 dt^4/4 0 dt^3/2; ...
        dt^3/2 0 dt^2 0; ...
        0 dt^3/2 0 dt^2].*HexAccel_noise_mag^2; % Ex convert the process noise (stdv) into covariance matrix
    P = Ex; % estimate of initial Hexbug position variance (covariance matrix)
    
    %% Define update equations in 2-D! (Coefficent matrices): A physics based model for where we expect the HEXBUG to be [state transition (state + velocity)] + [input control (acceleration)]
    A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]; %state update matrice
    B = [(dt^2/2); (dt^2/2); dt; dt];
    C = [1 0 0 0; 0 1 0 0];  %this is our measurement function C, that we apply to the state estimate Q to get our expect next/new measurement


    %% initize result variables
    % Initialize for speed
    Q_loc = []; % ACTUAL hexbug motion path
    vel = []; % ACTUAL hexbug velocity
    Q_loc_meas = []; % the hexbug path extracted by the tracking algo

    %% initize estimation variables
    Q_loc_estimate = []; %  position estimate
    vel_estimate = []; % velocity estimate
    P_estimate = P;
    predic_state = [];
    predic_var = [];
    total_data_after_kalman = [];
    
    for t = 1:required_runs                % The number of frames to process
        %Mathematical Model Starts Here
        % load the given tracking
        Q_loc_meas(:,t) = [CM_idx(t,1); CM_idx(t,2)];

        %% do the kalman filter   

        % Predict next state of the Hexbug with the last state and predicted motion.
        Q_estimate = A * Q_estimate + B * u;
        predic_state = [predic_state; Q_estimate(1)] ;
        %predict next covariance
        P = A * P * A' + Ex;
        predic_var = [predic_var; P] ;
        % predicted Ninja measurement covariance
        % Kalman Gain
        K = P*C'*inv(C*P*C'+Ez);
        % Update the state estimate.
        if ~isnan(Q_loc_meas(:,t))
            Q_estimate = Q_estimate + K * (Q_loc_meas(:,t) - C * Q_estimate);
        end
        % update covariance estimation.
        P =  (eye(4)-K*C)*P;
        %Mathematical Model Ends Here

        %Writing the file
        insert_row = [Q_loc_meas(1,t) Q_loc_meas(2,t) Q_estimate(1) Q_estimate(2) CM_idx(t,3)];
        total_data_after_kalman = [total_data_after_kalman;insert_row];
    end


end









