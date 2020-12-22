%% SETUP ENVIRONMENT
close all;
clc; clear;
addpath('./lib');

%% SIMULATED OBJECTS
drone1_init;
drone1 = Drone(drone1_params, drone1_initStates, drone1_Inputs, drone1_gains);

simTime = 2;

%% RUN SIMULATION
Animation;
                    