%% SETUP ENVIRONMENT
close all;
clc; clear;
addpath('./lib');


%% SIMULATED OBJECTS
initialize;
drone = Drone(drone_params, drone_initStates, drone_initInputs, drone_gains);


%% RUN SIMULATION
simTime = 5;
stepSize = 0.01;
iterNum = simTime/stepSize;

%commandSig = [deg2rad(6.0), deg2rad(10.0), deg2rad(0.0), 0]'; % phi theta psi zdot
commandSig = [3, 0, -5, deg2rad(0)]'; % x y z psi
sim_data = simulation(iterNum,drone,commandSig,1);


%% RUN ANIMATION
animation(sim_data, drone.body, iterNum);
                    