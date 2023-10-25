% Austin Perez
% mockCOMP

clear
clc
close all force
diary('off')
fclose('all') ;


vehiclefile = uigetfile('*.mat', 'Select a Vehicle File', './OpenVEHICLEVehicles') ;
veh = load(vehiclefile);



% Run Endurance
tr = load("./OpenTRACKTracks/OpenTRACK_Michigan Endurance_Closed_Forward.mat");
[sim] = simulate(veh,tr) ;
disp('========== ENDURANCE AND EFFICIENCY ==========');
disp(['Laptime:  ',num2str(sim.laptime.data,'%3.3f'),' [s]'])
disp(['Endurance Score:  ',num2str(sim.endurance.data,'%3.3f'),' [pts]'])
disp(['Efficiency Score:  ',num2str(sim.effScore.data,'%3.3f'),' [pts]'])

% Run Acceleration
time = accelerationSimulation(vehiclefile, 1);
DOO = 0; % Example number of cones knocked down
points = accelerationScore(time, DOO);
disp('========== ACCELERATION ==========');
disp(['Finished in: ', num2str(time),' [s]']);
disp(['Acceleration Score: ', num2str(points),' [pts]']);



% Run Autocross
tr = load("./OpenTRACKTracks/OpenTRACK_Autocross_Open_Forward.mat");
[sim] = simulate(veh,tr) ;
disp('========== AUTOCROSS ==========');
disp(['Laptime:  ',num2str(sim.laptime.data/10,'%3.3f'),' [s]'])
Tmin = 45.886; % Minimum time from the data
Tmax = 1.45 * Tmin; % 145% of Tmin
runTime = sim.laptime.data/10; % Example run time
DOO = 0; % Example number of displaced cones
OC = 0; % Example number of off courses

score = autocrossScore(runTime, DOO, OC, Tmin, Tmax);
disp(['Autocross Score:  ',num2str(score,'%3.3f'),' [pts]'])




% Run Skidpad
tr = load("./OpenTRACKTracks/OpenTRACK_Skidpad_Closed_Forward.mat");
[sim] = simulate(veh,tr) ;
disp('========== SKIDPAD ==========');
time = sim.laptime.data/40;
DOO = 0;              % Number of DOO (including entry and exit gate cones)
Tmin = 4.908;         % Lowest Corrected Time recorded for any team
if time < Tmin
    Tmin = time;
end

score = skidpadScore(time, DOO, Tmin);
disp(['Skidpad Score: ', num2str(score)]);
disp(['Laptime:  ',num2str(sim.laptime.data/40,'%3.3f'),' [s]'])



function score = autocrossScore(runTime, DOO, OC, Tmin, Tmax)
    % Calculate Corrected Time
    correctedTime = runTime + (DOO * 2) + (OC * 20);
    
    % Calculate Autocross Score based on Corrected Time and Tmin, Tmax
    if correctedTime < Tmax
        score = 118.5 * (Tmax / correctedTime) - 1 + 6.5 / ((Tmax / Tmin) - 1);
    else
        score = 6.5;
    end
end

function score = skidpadScore(time, DOO, Tmin)
    % Constants
    Tmax = 1.25 * Tmin;
    
    % Corrected Time Calculation
    correctedTime = (time) + (DOO * 0.125);
    
    % Score Calculation
    if correctedTime < Tmax
        score = 71.5 * ((Tmax / correctedTime)^2 - 1)/(((Tmax / Tmin)^2 - 1)) + 3.5 ;
      
    else
        score = 3.5;
    end
end

