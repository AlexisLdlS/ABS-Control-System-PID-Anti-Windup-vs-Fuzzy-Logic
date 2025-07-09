% ************************ INITIAL CONFIG SCRIPT ************************ %
% This script defines:                                                    %
% 1. Sampling period of the inputs to simulation and the controller.      %
% 2. Multiple model parameters.                                           %
% 3. Surface definition. Coefficients of fricition for the wheels         %
%    Two options:                                                         %
%    a. Fixed (constant) value for each wheel                             %
%    b. Enhanced split-mu                                                 %
% 4. The maneuver to be carried out by a ficticious driver, based on      %
%    throttle, brake and steering inputs.                                 %
% *********************************************************************** %

% *************+*********** MANEUVER DESCRIPTION ************************ %
% Panic brake apply at 70km/h                                             %
% No steering correction                                                  %
% Homogenous high-mu (0.9)                                                %
% *********************************************************************** %

% Close open windows, clear workspace and clean command window
close all;
clear;
clc;

% SAMPLING PERIOD (TIME STEP)
dT = 0.005;                     % (seconds)

% MODEL PARAMETERS
P_SteeringMax        = 20;      % Maximum wheel angle (deg)
P_SteeringRatio      = 7;       % Steering wheel : Wheel's steering angle
P_TorqueMax          = 600;     % Maximum axle torque (N*m)
P_BrakePressureMax   = 120;     % Maximum brake pressure (bar)
P_ABSPressureRateMax = 1200;    % Maximum ABS pressure rate (bar/s)
P_PedalTravelRateMax = 5;       % Maximum pedal travel rate (1/s)


% ************************** SURFACE DEFINITION ************************* %
% FIXED friction coefficients [mu_FL; mu_FR; mu_RL; mu_RR]
Mu = ones(4,1).*0.9;            % Homogenous high-mu
% Mu = [0.91 0.85 0.93 0.88]';    % Quasi-homogenous high-mu
% Mu = [0.9 0.6 0.9 0.6]';        % Fixed split-mu

% ENHANCED SPILT-MU FUNCTION 
% Coeffiecients of friction of the surfaces
MuSplit = [0.9 0.4]';           % [Left Right]

% Function parameters
P_SplitMu_On           = false; % Enhanced Split-mu function switch
P_SplitMuOnlyAtBraking = false; % Enhanced Split-mu only at braking switch
                                % TRUE: Fixed Mu is used prior to braking
P_RefLine4SplitMue     = 6.14;  % Global X or Y coordinate (m)
P_RefLineIsX           = false; % Switch between X or Y split-mu
P_RefLine_On           = false; % Show a reference line in the XY PLOTTER
                                % to indicate the split-mu division
                                % Details in 'Main/Visualization/RefLine'

% ************************ SURFACE DEFINITION - END ********************* %



% ************************* MANEUVER DEFINITION ************************* %
% Maneuver's duration (not to be confused with Simulink's Stop-Time)
Tf = 20;

% Define Time column vector
t  = 0:dT:Tf;
t  = t';

% % Define pedals behavior over time
% % t - time (s)
% % k - position in vector corresponding to time t
% Throttle ramp up (from time 0 to t_RampUp)
t_RampUp = 1;
k_RampUp = t_RampUp/dT + 1;
% Wide Open Throttle (from time t_RampUp to t_WOT)
t_WOT = 6.7;
k_WOT = k_RampUp + t_WOT/dT +1;
% Brake apply (from time t_Brake to Tf)
t_Brake = 12;
k_Brake = t_Brake/dT + 1;
% % END of pedals behavior over time definition

% Define and build column vectors of throttle and brake pedals positions 
ThrPos = zeros(length(t),1);
BrkPos = zeros(length(t),1);
% Throttle
ThrPos(1:k_RampUp) = t(1:k_RampUp).^2 / (t_RampUp^2);
ThrPos(k_RampUp+1:k_WOT) = 1;
ThrPos(k_WOT+1:k_Brake) = 0.1;
% Brake
BrkPos(k_Brake+1:end) = 1;


% % Define steering over time
t_SteerInit = 6;
t_SteerStop = 11;
k_SteerInit = t_SteerInit/dT + 1;
k_SteerStop = t_SteerStop/dT + 1;
% Define and build column vector of steering wheel inputs
StrPos = zeros(length(t),1);
% Steering wheel input (comment lines below to have a constant input of 0)
% temp = linspace(0, 2*pi , (k_SteerStop-k_SteerInit)+1)';
% StrPos(k_SteerInit:k_SteerStop) = 7 * sin(temp);


% Build data for Simulink
Drv_ThrPedal = [t ThrPos];
Drv_BrkPedal = [t BrkPos];
Drv_Steering = [t StrPos];
% ********************** MANEUVER DEFINITION - END ********************** %


% **************************** PLOT MANEUVER **************************** %
figure;
% Plot Throttle
subplot(3,1,1);
plot(t,ThrPos, 'Linewidth', 2);
ylim([0 1]);
title('Throttle Pedal');
xlabel('Time (s)');
ylabel('Position');
% Plot Brake
subplot(3,1,2);
plot(t,BrkPos, 'Linewidth', 2);
ylim([0 1]);
xlabel('Time (s)');
ylabel('Position');
title('Brake Pedal');
% Plot Steering
subplot(3,1,3);
plot(t,StrPos, 'Linewidth', 2);
title('Steering Wheel');
xlabel('Time (s)');
ylabel('Angle (deg)');
% ************************* PLOT MANEUVER - END ************************* %
