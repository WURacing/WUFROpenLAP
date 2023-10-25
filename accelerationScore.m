function score = accelerationScore(time, DOO)
    % Constants
    Tmin = 4.174; % Minimum time from the provided data
    Tmax = 1.5 * Tmin; % 150% of Tmin

    % Calculate Corrected Time
    if time < Tmin
        Tmin = time;
    end
    Tyour = time + (DOO * 2);

    % Calculate Score
    if Tyour < Tmax
        score = 95.5 * (Tmax / Tyour - 1) / (Tmax / Tmin - 1) + 4.5;
    else
        score = 4.5;
    end
end