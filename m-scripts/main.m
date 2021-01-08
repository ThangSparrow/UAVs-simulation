%% SETUP ENVIRONMENT
close all;
clc; clear;
addpath('./lib');


%% SIMULATED OBJECTS
drone1_init;
drone1 = Drone(drone1_params, drone1_initStates, drone1_initInputs, drone1_gains);


%% RUN SIMULATION
simTime = 2;
stepSize = 0.01;
iterNum = simTime/stepSize;

commandSig = [1, 3, -5, 0]'; % phi theta psi zdot
sim_data = simulation(iterNum,drone1,commandSig);


%% RUN ANIMATION
animation(sim_data, drone1.body, iterNum);
                    