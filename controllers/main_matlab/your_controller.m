classdef your_controller < BaseController
    % Project 4 student template: controller plus EKF-SLAM integration.
    properties
        lr = 0.82
        lf = 1.18
        Ca = 20000
        Iz = 3004.5
        m = 1000
        g = 9.81
        counter = 0
        map
        n
        slam
        previousU = [0; 0; 0]
    end

    methods
        function obj = your_controller(trajectory)
            obj@BaseController(trajectory);
            rng(99, 'twister');
        end

        function [delT, X, Y, xdot, ydot, psi, psidot] = getEstimatedStates(obj, timestep, useSlam)
            [delT, X, Y, xdot, ydot, psi, psidot] = obj.getStates(timestep);
            if obj.counter == 0
                [mapX, mapY] = meshgrid(linspace(-120, 450, 7), linspace(-350, 50, 7));
                obj.map = reshape([mapX(:), mapY(:)]', [], 1);
                obj.n = numel(obj.map) / 2;
                initialMean = [X + 0.5; Y - 0.5; psi - 0.02; obj.map];
                initialCovariance = eye(3 + 2 * obj.n);
                processNoise = zeros(3 + 2 * obj.n);
                processNoise(1:3, 1:3) = delT^2 * 0.1 * eye(3);
                observationNoise = 0.1 * eye(2 * obj.n);
                observationNoise(obj.n + 1:end, obj.n + 1:end) = 0.01 * eye(obj.n);
                obj.slam = EKF_SLAM(initialMean, initialCovariance, delT, ...
                    processNoise, observationNoise, obj.n);
                obj.counter = 1;
            else
                measurements = obj.computeMeasurements(X, Y, psi);
                [estimate, ~] = obj.slam.predictAndCorrect(measurements, obj.previousU);
                if useSlam
                    X = estimate(1); Y = estimate(2); psi = estimate(3);
                end
            end
            obj.previousU = [xdot; ydot; psidot];
        end

        function y = computeMeasurements(obj, X, Y, psi)
            landmarks = reshape(obj.map, 2, [])';
            offsets = landmarks - [X, Y];
            ranges = sqrt(sum(offsets .^ 2, 2));
            bearings = wrapToPi(atan2(offsets(:, 2), offsets(:, 1)) - psi);
            noise = chol(obj.slam.V, 'lower') * randn(2 * obj.n, 1);
            y = [ranges; bearings] + noise;
        end

        function [X, Y, xdot, ydot, psi, psidot, F, delta] = update(obj, timestep)
            [delT, X, Y, xdot, ydot, psi, psidot] = ...
                obj.getEstimatedStates(timestep, true); %#ok<ASGLU>
            trajectory = obj.trajectory; %#ok<NASGU>

            % ---------------|Lateral Controller|-------------------------
            % Reuse or refine your controller from Project 3. Assign delta.
            delta = 0;

            % ---------------|Longitudinal Controller|--------------------
            % Reuse or refine your controller from Project 3. Assign F.
            F = 0;
        end
    end
end
