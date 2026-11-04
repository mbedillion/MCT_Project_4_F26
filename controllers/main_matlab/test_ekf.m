function test_ekf
    % Standalone checks for the student EKF-SLAM implementation.
    % Run from MATLAB after adding this controller directory to the path.
    rng(7);
    n = 4;
    dt = 0.032;
    x = [2; -3; 0.4; 0; 0; 10; 2; -4; 8; 7; -6];
    P = eye(3 + 2*n);
    W = zeros(size(P));
    W(1:3, 1:3) = 1e-4 * eye(3);
    V = diag([0.1*ones(1, n), 0.01*ones(1, n)]);
    filter = EKF_SLAM(x, P, dt, W, V, n);
    u = [5; -0.2; 0.3];

    epsilon = 1e-6;
    numericF = zeros(size(P));
    for column = 1:numel(x)
        offset = zeros(size(x));
        offset(column) = epsilon;
        numericF(:, column) = ...
            (filter.f(x + offset, u) - filter.f(x - offset, u)) / (2*epsilon);
    end
    assert(max(abs(filter.computeF(x, u) - numericF), [], 'all') < 1e-6, ...
        'F Jacobian mismatch.');

    numericH = zeros(2*n, numel(x));
    for column = 1:numel(x)
        offset = zeros(size(x));
        offset(column) = epsilon;
        difference = filter.h(x + offset) - filter.h(x - offset);
        difference(n + 1:end) = wrapToPi(difference(n + 1:end));
        numericH(:, column) = difference / (2*epsilon);
    end
    assert(max(abs(filter.computeH(x) - numericH), [], 'all') < 1e-5, ...
        'H Jacobian mismatch.');

    trueState = filter.f(x, u);
    y = filter.h(trueState);
    [mu, covariance] = filter.predictAndCorrect(y, u);
    assert(all(isfinite(mu)) && all(isfinite(covariance), 'all'), ...
        'The EKF produced a nonfinite estimate.');
    assert(norm(covariance - covariance', 'fro') < 1e-10, ...
        'The covariance is not symmetric.');
    assert(min(eig(covariance)) > -1e-10, ...
        'The covariance is not positive semidefinite.');

    fprintf('EKF unit test passed; max F error %.3g, max H error %.3g.\n', ...
        max(abs(filter.computeF(x, u) - numericF), [], 'all'), ...
        max(abs(filter.computeH(x) - numericH), [], 'all'));
end
