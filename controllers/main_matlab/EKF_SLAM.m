classdef EKF_SLAM < handle
    % Student implementation of the Project 4 EKF-SLAM estimator.
    properties
        mu
        P
        dt
        W
        V
        n
    end

    methods
        function obj = EKF_SLAM(initMu, initP, dt, W, V, n)
            obj.mu = initMu(:);
            obj.P = initP;
            obj.dt = dt;
            obj.W = W;
            obj.V = V;
            obj.n = n;
        end

        function xNext = f(obj, x, u) %#ok<INUSD>
            % TODO: implement the nonlinear motion model from Equation (2).
            % Preserve the static landmark states and wrap only heading.
            xNext = x;
        end

        function y = h(obj, x) %#ok<INUSD>
            % TODO: implement the stacked n ranges followed by n bearings.
            y = zeros(2 * obj.n, 1);
        end

        function F = computeF(obj, x, u) %#ok<INUSD>
            % TODO: implement the full (3 + 2*n)-state motion Jacobian.
            F = eye(3 + 2 * obj.n);
        end

        function H = computeH(obj, x) %#ok<INUSD>
            % TODO: implement the 2*n by (3 + 2*n) measurement Jacobian.
            H = zeros(2 * obj.n, 3 + 2 * obj.n);
        end

        function [mu, P] = predictAndCorrect(obj, y, u) %#ok<INUSD>
            % TODO: implement EKF prediction followed by correction. Wrap
            % only the bearing innovations and the estimated heading.
            mu = obj.mu;
            P = obj.P;
        end
    end
end

