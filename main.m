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

commandSig = [deg2rad(10), deg2rad(10), deg2rad(30), 0]'; % phi theta psi zdot
sim_data = simulation(iterNum,drone1,commandSig);


%% RUN ANIMATION
animation(sim_data, drone1.body, iterNum);
                    