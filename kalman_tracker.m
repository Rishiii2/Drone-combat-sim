classdef kalman_tracker < handle
% KALMAN_TRACKER  2-D Kalman Filter (constant-velocity model)
%   State  : [x; y; vx; vy]
%   Measure: [x; y]
%   Track lifecycle: hits / lost counters for DeepSORT-style management

    properties
        x_est       % State estimate  [x; y; vx; vy]
        P_est       % Covariance matrix (4x4)
        A           % State-transition matrix
        H           % Observation matrix
        Q           % Process-noise covariance
        R           % Measurement-noise covariance
        ID          % Unique integer ID
        hits        % Consecutive matched updates
        lost        % Consecutive missed frames
        confirmed   % Boolean - track is reliable
        age         % Total frames since creation
    end

    properties (Constant)
        MIN_HITS = 1    % FIX: was 2, now 1 so tracks confirm immediately
        MAX_LOST = 8    % frames before track is deleted
    end

    methods
        function obj = kalman_tracker(init_x, init_y, id)
            dt = 0.1;
            obj.ID        = id;
            obj.hits      = 1;
            obj.lost      = 0;
            obj.confirmed = true;   % FIX: confirm immediately on creation
            obj.age       = 1;

            obj.A = [1 0 dt  0;
                     0 1  0 dt;
                     0 0  1  0;
                     0 0  0  1];

            obj.H = [1 0 0 0;
                     0 1 0 0];

            obj.Q = diag([0.05, 0.05, 0.5, 0.5]);
            obj.R = eye(2) * 1.5;
            obj.P_est = diag([5, 5, 50, 50]);
            obj.x_est = [init_x; init_y; 0; 0];
        end

        function predict(obj)
            obj.x_est = obj.A * obj.x_est;
            obj.P_est = obj.A * obj.P_est * obj.A' + obj.Q;
            obj.age   = obj.age + 1;
        end

        function update(obj, measurement)
            % measurement: [x; y] column vector
            y_res = measurement - obj.H * obj.x_est;
            S     = obj.H * obj.P_est * obj.H' + obj.R;
            K     = obj.P_est * obj.H' / S;
            obj.x_est = obj.x_est + K * y_res;
            obj.P_est = (eye(4) - K * obj.H) * obj.P_est;
            obj.hits  = obj.hits + 1;
            obj.lost  = 0;
            if obj.hits >= obj.MIN_HITS
                obj.confirmed = true;
            end
        end

        function mark_missed(obj)
            obj.lost = obj.lost + 1;
        end

        function tf = should_delete(obj)
            tf = obj.lost >= obj.MAX_LOST;
        end

        function pos = get_position(obj)
            pos = obj.x_est(1:2)';  % [x, y] row vector
        end

        function vel = get_velocity(obj)
            vel = obj.x_est(3:4)';  % [vx, vy] row vector
        end
    end
end
