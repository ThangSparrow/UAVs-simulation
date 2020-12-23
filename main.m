%% SETUP ENVIRONMENT
close all;
clc; clear;
addpath('./lib');

%% SIMULATED OBJECTS
drone1_init;
drone1 = Drone(drone1_params, drone1_initStates, drone1_Inputs, drone1_gains);

simTime = 1;

%% RUN SIMULATION
commandSig(1) = deg2rad(10);
commandSig(2) = deg2rad(10);
commandSig(3) = deg2rad(30);
commandSig(4) = 0.0;
Animation;
                    