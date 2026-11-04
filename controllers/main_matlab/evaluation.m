function [grade, scores] = evaluation(minDistList, traj_, X, Y, deltaVec)
    fprintf('Evaluating...\n');
    % Fall 2026 assignment thresholds (MCT_Project_4_2026.pdf).
    timeBaseline = 200; dt = 0.032; Cmax_cl = 15.0; Cavg = 5.0;
    Cmax_md = 10.0; deltaThreshold = 0.025; fs = 22.5;
    traj = traj_(2:end-60, :);
    comGrad = clGrader(traj, X, Y, fs, Cmax_cl);
    avgGrad = adGrader(minDistList, fs, Cavg);
    maxGrad = fs * sum(minDistList <= Cmax_md) / length(minDistList);
    averageDeltaChange = mean(abs(diff(deltaVec)));
    if averageDeltaChange <= deltaThreshold
        deltaGrad = fs;
    elseif averageDeltaChange <= 2 * deltaThreshold
        deltaGrad = 0.5 * fs;
    else
        deltaGrad = 0;
    end
    fprintf('Score for completing the loop: %.2f/%.2f\n', comGrad, fs);
    fprintf('Score for average distance: %.2f/%.2f\n', avgGrad, fs);
    fprintf('Score for maximum distance: %.2f/%.2f\n', maxGrad, fs);
    fprintf('Average steering-angle change: %.6f rad/timestep\n', averageDeltaChange);
    fprintf('Score for average steering-angle change: %.2f/%.2f\n', deltaGrad, fs);
    speedScore = 0;
    if comGrad == fs
        elapsed = length(X) * dt;
        if elapsed <= timeBaseline, speedScore = 10;
        elseif elapsed <= 2 * timeBaseline, speedScore = 20 - 10 * elapsed / timeBaseline; end
        fprintf('Your time is %.4f\n', elapsed);
    else
        fprintf('Your vehicle did not finish the loop.\nYou cannot enter the competition.\n');
    end
    grade = comGrad + avgGrad + maxGrad + deltaGrad + speedScore;
    scores = [comGrad, avgGrad, maxGrad, deltaGrad, speedScore];
    fprintf('Your total score is : %.2f/100.0\n', grade);
end

function score = clGrader(traj, X, Y, fs, threshold)
    samples = [X(:), Y(:)]; count = 0;
    for first = 1:256:size(traj, 1)
        last = min(first + 255, size(traj, 1));
        dx = traj(first:last, 1) - samples(:, 1)';
        dy = traj(first:last, 2) - samples(:, 2)';
        count = count + sum(sqrt(min(dx.^2 + dy.^2, [], 2)) <= threshold);
    end
    score = fs * count / size(traj, 1);
end

function score = adGrader(values, fs, threshold)
    average = mean(values);
    if average <= threshold
        score = fs;
    elseif average <= 2 * threshold
        score = (-20 / threshold) * average + 40;
    else
        score = 0;
    end
end

