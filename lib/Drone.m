classdef Drone < handle
%% DYNAMICS
    properties
        g
        % simTime
        dt
        % endTime
        
        m
        l
        I
        
        x                   % [X, Y, Z, dX, dY, dZ, phi, theta, psi, p, q, r]
        r                   % [X, Y, Z] position vector
        dr                  % [dX, dY, dZ] 
        euler               % [phi, theta, psi]' - Attitude
        w                   % [p, q, r]' - Angular rates in body frame
        
        dx
                    
        u                   % [T_sum, M1, M2 ,M3]'
        T                   % T_sum
        M                   % [M1, M2, M3]'
    end
    
%% INPUT
    properties
        phi_des
        phi_err
        phi_err_prev
        phi_err_sum
        
        theta_des
        theta_err
        theta_err_prev
        theta_err_sum
        
        psi_des
        psi_err
        psi_err_prev
        psi_err_sum
        
        zdot_des
        zdot_err
        zdot_err_prev
        zdot_err_sum
        
        kP_phi
        kI_phi
        kD_phi
        
        kP_theta
        kI_theta
        kD_theta
        
        kP_psi
        kI_psi
        kD_psi
        
        kP_zdot
        kI_zdot
        kD_zdot
    end

%% METHODS
    methods
        %% CONSTRUCTOR
        function obj = Drone(params, initStates, Inputs, gains) % , endTime)
            obj.g = 9.81;
            % obj.simTime = 0.0;
            obj.dt = 0.01;
            % obj.endTime = endTime;

            obj.m = params('Mass');
            obj.l = params('armLength');
            obj.I = [params('Ixx'),             0,             0; ...
                                 0, params('Iyy'),             0; ...
                                 0,             0,  params('Izz')];

            obj.x = initStates;                   
            obj.r = obj.x(1:3);  
            obj.dr = obj.x(4:6);
            obj.euler = obj.x(7:9);
            obj.w = obj.x(10:12);

            obj.dx = zeros(12,1);

            obj.u = Inputs;
            obj.T = obj.u(1);
            obj.M = obj.u(2:4);
            
            obj.phi_des = 0.0;
            obj.phi_err = 0.0;
            obj.phi_err_prev = 0.0;
            obj.phi_err_sum = 0.0;

            obj.theta_des = 0.0;
            obj.theta_err = 0.0;
            obj.theta_err_prev = 0.0;
            obj.theta_err_sum = 0.0;

            obj.psi_des = 0.0;
            obj.psi_err = 0.0;
            obj.psi_err_prev = 0.0;
            obj.psi_err_sum = 0.0;

            obj.zdot_des = 0.0;
            obj.zdot_err = 0.0;
            obj.zdot_err_prev = 0.0;
            obj.zdot_err_sum = 0.0;

            obj.kP_phi = gains('P_phi');
            obj.kI_phi = gains('I_phi');
            obj.kD_phi = gains('D_phi');
            
            obj.kP_theta = gains('P_theta');
            obj.kI_theta = gains('I_theta');
            obj.kD_theta = gains('D_theta');
            
            obj.kP_psi = gains('P_psi');
            obj.kI_psi = gains('I_psi');
            obj.kD_psi = gains('D_psi');
            
            obj.kP_zdot = gains('P_zdot');
            obj.kI_zdot = gains('I_zdot');
            obj.kD_zdot = gains('D_zdot');
        end
        
        function state = GetState(obj)
            state = obj.x;
        end
        
        function obj = EvalEOM(obj)
            obj.dx(1:3) = obj.dr;
            
            bRi = RPY2Rot(obj.euler);
            R = bRi';
            obj.dx(4:6) = 1/obj.m * (-R * obj.T * [0;0;-1] + [0;0; obj.m*obj.g]);
            
            s_p = sin(obj.euler(1)); c_p = cos(obj.euler(1));
            t_t = tan(obj.euler(2)); sec_t = sec(obj.euler(2));
            obj.dx(7:9) = [1    s_p*t_t        c_p*t_t; ...
                           0    c_p            -s_p; ...
                           0    s_p*sec_t      c_p*sec_t] * obj.w;
            obj.dx(10:12) = (obj.I) \ (obj.M - cross(obj.w, obj.I * obj.w));
        end 
        
        function obj = UpdateState(obj)
            % obj.simTime = obj.simTime + obj.dt;
            
            obj.EvalEOM();
            obj.x = obj.x + obj.dx*obj.dt;    % Euler integrate method
        
            obj.r = obj.x(1:3);
            obj.dr = obj.x(4:6);
            obj.euler = obj.x(7:9);
            obj.w = obj.x(10:12);
        end
        
        function obj = AttitudeCtrl(obj, refSig)
            obj.u(1) = refSig(1);
            obj.u(2) = refSig(2);
            obj.u(3) = refSig(3);
            obj.u(4) = refSig(4);
            
            obj.T = obj.u(1);
            obj.M = obj.u(2:4);
        end
    end
end