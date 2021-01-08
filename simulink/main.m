%% SETUP ENVIRONMENT
close all;
clc; clear;
addpath('./lib');


%% Initializing model
Initialize;
simTime = 2;
stepSize = 0.01;


%% RUN SIMULATION
model = 'Drone.slx';
open_system(model);

commandSig = [1, 3, -5, deg2rad(0)]'; % X Y Z psi
sim = sim(model, simTime);
X_sim = sim.get('X_state');


%% PLOT RUN ANIMATION
plot_script(X_sim,commandSig,stepSize);
% animation(sim_data, drone1.body, stepSize);
                    