clc; clear; close all;

%% 1. Parametre og Banegenerering

% --- NYT: Filopsætning ---
filename = 'test_results.csv';

% Tjek om filen eksisterer og slet den for at starte forfra
if exist(filename, 'file')
    delete(filename);
end

% Åbn filen for skrivning ('a' = append, men vi sletter den først)
fileID = fopen(filename, 'a'); 

% Skriv overskriftsrækken
fprintf(fileID, 'releaseX,releaseY,releaseZ,targetX,targetY,targetZ,status\n'); 
% --- Slut på Filopsætning ---

followTime = 0.75;
frequency = 125; % Sampling/Animationsfrekvens (Hz)
fileNumber = 321;
transformW2R = [-0.3840, -0.9233, 0.0005,  0.7021;
                 0.9233, -0.3840, 0.0018, -0.7111;
                -0.0015,  0.0012, 1.0000, -0.0334;
                 0.0000,  0.0000, 0.0000,  1.0000];

% ----------------------------------------------------
% --- PARALLELISERING START ---
% ----------------------------------------------------

% Definer parameterrummet og beregn totalt antal tests.
rx_vals = 9:12; disp(rx_vals*0.05);
ry_vals = 6:10; disp(ry_vals*0.05);
rz_vals = 7:10; disp(rz_vals*0.05);
tx_vals = -4:2; disp(tx_vals*0.05);
ty_vals = 2:6; disp(ty_vals*0.1);

jointAcceleration = 7;

num_rx = length(rx_vals);
num_ry = length(ry_vals);
num_rz = length(rz_vals);
num_tx = length(tx_vals);
num_ty = length(ty_vals);

TOTAL_ITERATIONS = num_rx * num_ry * num_rz * num_tx * num_ty;
disp(TOTAL_ITERATIONS);

%%
% Initialiser et array til at gemme alle input-kombinationer
% og initialiser output (status) til at blive samlet efter kørsel.
input_combinations = zeros(TOTAL_ITERATIONS, 6); % releaseX,Y,Z, targetX,Y,Z
output_status = zeros(TOTAL_ITERATIONS, 1);       % status

iteration_idx = 1;

for rx = rx_vals
for ry = ry_vals
for rz = rz_vals
for tx = tx_vals
for ty = ty_vals
    
    releasePosition = [0.05*rx 0.05*ry 0.05*rz];
    targetPosition = [0.05*tx 0.1*ty 0.0];
    
    input_combinations(iteration_idx, 1:3) = releasePosition;
    input_combinations(iteration_idx, 4:6) = targetPosition;
    
    iteration_idx = iteration_idx + 1;
end
end
end
end
end

% Transformationen er konstant og defineres før parallel-loop
transformW2R = [-0.3840, -0.9233, 0.0005,  0.7021;
                 0.9233, -0.3840, 0.0018, -0.7111;
                -0.0015,  0.0012, 1.0000, -0.0334;
                 0.0000,  0.0000, 0.0000,  1.0000];

% Opret den parallelle pool. (MATLAB starter typisk en worker pr. CPU-kerne)
mypool = parpool; 

ppm = ParforProgressbar(TOTAL_ITERATIONS, "showWorkerProgress", true);

% Hovedløkken paralleliseres
parfor i = 1:TOTAL_ITERATIONS

    % Hent input for denne iteration
    releasePosition = input_combinations(i, 1:3);
    targetPosition = input_combinations(i, 4:6);

    if norm(releasePosition(1:2) - targetPosition(1:2)) > 0.30
        % Beregn bane og hastighed
        [yaw, pitch, releaseVelocity] = trajectory(releasePosition, targetPosition);
    
        % Kør ekstern funktion for at generere ledvinkler (q) og hastigheder (qd)
        [status, q, qd] = throwFunction(releasePosition, targetPosition, yaw, pitch, releaseVelocity, jointAcceleration, followTime, frequency, transformW2R, fileNumber);
    else
        status = 26;
    end
    
    % Gem kun status i det pre-allokerede output array
    output_status(i) = status;
    
    ppm.increment();
    
end

delete(ppm);

% Luk den parallelle pool.
delete(mypool);

% ----------------------------------------------------
% --- PARALLELISERING SLUT ---
% 
% --- 2. Resultatsammenfatning og Filskrivning ---
% ----------------------------------------------------

disp('Parallelle beregninger afsluttet. Skriver til fil...');

% Iterér gennem de indsamlede resultater og skriv til fil
for i = 1:TOTAL_ITERATIONS
    
    current_releasePos = input_combinations(i, 1:3);
    current_targetPos = input_combinations(i, 4:6);
    current_status = output_status(i);
    
    fprintf(fileID, '%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.0f\n', ...
        current_releasePos(1), current_releasePos(2), current_releasePos(3), ...
        current_targetPos(1), current_targetPos(2), current_targetPos(3), ...
        current_status);
        
    % Hvis du stadig ønsker at se output i konsollen
    % fprintf('Test %d - Status: %.0f\n', i, current_status); 
end

% Luk filen når alle tests er færdige
fclose(fileID); 
disp(['Resultater er gemt i ' filename]);

function [yaw, pitch, velocity] = trajectory(releasePosition, targetPosition)
% TRAJECTORY Finder den minimale starthastighed og de tilhørende vinkler
% (yaw og pitch) for at ramme et mål.
%   yaw og pitch returneres nu i RADIANER.
%
%   releasePosition: [x_r, y_r, z_r] - Startposition (m)
%   targetPosition: [x_t, y_t, z_t] - Målposition (m)
    % --- Konstanter ---
    g = 9.81; % Tyngdeacceleration (m/s^2)
    minPitch_deg = 22.5; % Minimum pitch-vinkel (grader, bruges KUN til beregning)
    
    % --- Bestem Forskydning ---
    delta_r = targetPosition - releasePosition;
    
    % Horisontal afstand (i xy-planet)
    R_xy = sqrt(delta_r(1)^2 + delta_r(2)^2);
    
    % Vertikal afstand (z-akse)
    delta_z = delta_r(3); 
    
    if R_xy == 0
        fprintf('[Matlab error] Target is directly under release point\n');
        return
    end
    % --- 1. Bestem Yaw (Resultat i RADIANER) ---
    % Yaw: 0 rad i -x retning. Positiv mod +y, Negativ mod -y.
    
    theta_from_plus_x = atan2(delta_r(2), delta_r(1)); % Vinkel fra +x-aksen
    
    % Vinklen fra den NEGATIVE X-akse (wrappet til [-pi, pi])
    yaw = pi - theta_from_plus_x;
    yaw = mod(yaw + pi, 2*pi) - pi; % yaw er nu i radianer [-pi, pi]
    
    % --- 2. Optimer Pitch og Velocity (Resultat i RADIANER) ---
    
    minPitch_rad = deg2rad(minPitch_deg);
    
    % Søg efter pitch i intervallet [minPitch, 89.99 grader]
    pitch_search_rad = linspace(minPitch_rad, deg2rad(89.99), 1000); 
    
    v0_squared = zeros(size(pitch_search_rad));
    
    for i = 1:length(pitch_search_rad)
        theta = pitch_search_rad(i);
        
        % V0^2 ligningen:
        % v0^2 = (g * R_xy^2) / (2 * cos(theta)^2 * (R_xy * tan(theta) - delta_z))
        denominator = 2 * cos(theta)^2 * (R_xy * tan(theta) - delta_z);
        
        if denominator > 1e-6 
            v0_squared(i) = (g * R_xy^2) / denominator;
        else
            v0_squared(i) = Inf; 
        end
    end
    
    % Find den minimale hastighed
    [min_v0_squared, min_idx] = min(v0_squared);
    
    velocity = sqrt(min_v0_squared);
    
    % Tildel optimal pitch i radianer
    pitch = pitch_search_rad(min_idx); 
    
    % Håndter kravet om minimum pitch (nu i radianer)
    if pitch < minPitch_rad
       pitch = minPitch_rad; % Brug minPitch i radianer
       
       % Genberegn hastighed for den påkrævede minimum pitch
       denominator_min = 2 * cos(pitch)^2 * (R_xy * tan(pitch) - delta_z);
       if denominator_min > 0
           velocity = sqrt((g * R_xy^2) / denominator_min);
       else
           fprintf('[Matlab error] Cannot hit target with minimum angle\n'); 
       end
    end
end