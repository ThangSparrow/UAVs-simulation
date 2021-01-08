classdef Drone < handle
%% DYNAMICS
    properties
        body
        
        g
        dt
        
        m
        l
        I
        
        x                   % [X, Y, Z, dX, dY, dZ, phi, theta, psi, p, q, r]
        r                   % [X, Y, Z] position vector
        dr                  % [dX, dY, dZ] 
        euler               % [phi, theta, psi]' - Attitude
        w                   % [p, q, r]' - Angular rates in body frame
        
        dx
                    
        u_attitude          % [T_sum, M1, M2 ,M3]'
        u_position          % [X, Y, Z, psi]'
        T                   % T_sum
        M                   % [M1, M2, M3]'
    end
    
%% INPUT
    properties
        %% Attitude Control
        attitude_cmd
        
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
        
        %% Position Control
        x_des
        x_err
        x_err_prev
        x_err_sum
        
        x_err_b
        
        y_des
        y_err
        y_err_prev
        y_err_sum
        
        y_err_b
        
        z_des
        z_err
        z_err_prev
        z_err_sum
        
        kP_x
        kI_x
        kD_x
        
        kP_y
        kI_y
        kD_y
        
        kP_z
        kI_z
        kD_z
    end

%% METHODS
    methods
        %% CONSTRUCTOR
        function obj = Drone(params, initStates, initInputs, gains)
            obj.g = 9.81;
            obj.dt = 0.01;

            obj.m = params('Mass');
            obj.l = params('armLength');
            obj.I = [params('Ixx'),             0,             0; ...
                                 0, params('Iyy'),             0; ...
                                 0,             0,  params('Izz')];
                             
            obj.body = [ obj.l,      0,     0,   1; ...  % first arm
                             0, -obj.l,     0,   1; ...  % second arm
                        -obj.l,      0,     0,   1; ...  % third arm
                             0,  obj.l,     0,   1; ...  % fouth arm
                             0,      0,     0,   1; ...  % center
                             0,      0, -0.15,   1]';    % controller

            obj.x = initStates;                   
            obj.r = obj.x(1:3);  
            obj.dr = obj.x(4:6);
            obj.euler = obj.x(7:9);
            obj.w = obj.x(10:12);

            obj.dx = zeros(12,1);

            obj.u_attitude = initInputs;
            obj.T = obj.u_attitude(1);
            obj.M = obj.u_attitude(2:4);
            
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
            
            %% Initialize Position Controler
            obj.x_des = 0.0;
            obj.x_err = 0.0;
            obj.x_err_prev = 0.0;
            obj.x_err_sum = 0.0;
            
            obj.x_err_b = 0.0;

            obj.y_des = 0.0;
            obj.y_err = 0.0;
            obj.y_err_prev = 0.0;
            obj.y_err_sum = 0.0;
            
            obj.y_err_b = 0.0;

            obj.z_des = 0.0;
            obj.z_err = 0.0;
            obj.z_err_prev = 0.0;
            obj.z_err_sum = 0.0;

            obj.kP_x = gains('P_x');
            obj.kI_x = gains('I_x');
            obj.kD_x = gains('D_x');
            
            obj.kP_y = gains('P_y');
            obj.kI_y = gains('I_y');
            obj.kD_y = gains('D_y');
            
            obj.kP_z = gains('P_z');
            obj.kI_z = gains('I_z');
            obj.kD_z = gains('D_z');
            
        end
        
        function state = GetState(obj)
            state = obj.x;
        end
        
        function obj = EvalEOM(obj)
            obj.dx(1:3) = obj.dr;
            
            bRi = RPY2Rot(obj.euler);
            R = bRi';
            obj.dx(4:6) = 1 / obj.m * (R * obj.T * [0;0;-1] + [0;0; obj.m*obj.g]);
            
            phi = obj.euler(1); theta = obj.euler(2);
            obj.dx(7:9) = [1    sin(phi)*tan(theta)        cos(phi)*tan(theta);
                           0    cos(phi)                   -sin(phi);
                           0    sin(phi)*sec(theta)        cos(phi)*sec(theta)] * obj.w;
            obj.dx(10:12) = (obj.I) \ (obj.M - cross(obj.w, obj.I * obj.w));
        end 
        
        function obj = UpdateState(obj)
            obj.EvalEOM();
            obj.x = obj.x + obj.dx*obj.dt;    % Euler integrate method
        
            obj.r = obj.x(1:3);
            obj.dr = obj.x(4:6);
            obj.euler = obj.x(7:9);
            obj.w = obj.x(10:12);
        end
        
        function obj = AttitudeCtrl(obj, attitude_cmd)
            if attitude_cmd == 0
                obj.phi_des = obj.attitude_cmd(1);
                obj.theta_des = obj.attitude_cmd(2);
                obj.psi_des = obj.attitude_cmd(3);
                obj.zdot_des = obj.attitude_cmd(4);
            else
                obj.phi_des = attitude_cmd(1);
                obj.theta_des = attitude_cmd(2);
                obj.psi_des = attitude_cmd(3);
                obj.zdot_des = attitude_cmd(4);
            end
            
            obj.phi_err = obj.phi_des - obj.euler(1);
            obj.theta_err = obj.theta_des - obj.euler(2);
            obj.psi_err = obj.psi_des - obj.euler(3);
            obj.zdot_err = obj.zdot_des - obj.dr(3);
            
            obj.u_attitude(2) = (obj.kP_phi * obj.phi_err + ...
                        obj.kI_phi * obj.phi_err_sum + ...
                        obj.kD_phi * (obj.phi_err - obj.phi_err_prev)/obj.dt);
            obj.phi_err_sum = obj.phi_err_sum + obj.phi_err;
            obj.phi_err_prev = obj.phi_err;
            
            obj.u_attitude(3) = (obj.kP_theta * obj.theta_err + ...
                        obj.kI_theta * obj.theta_err_sum + ...
                        obj.kD_theta * (obj.theta_err - obj.theta_err_prev)/obj.dt);        
            obj.theta_err_sum = obj.theta_err_sum + obj.theta_err;
            obj.theta_err_prev = obj.theta_err;
            
            obj.u_attitude(4) = (obj.kP_psi * obj.psi_err + ...
                        obj.kI_psi * obj.psi_err_sum + ...
                        obj.kD_psi * (obj.psi_err - obj.psi_err_prev)/obj.dt);           
            obj.psi_err_sum = obj.psi_err_sum + obj.psi_err;
            obj.psi_err_prev = obj.psi_err;
            
            obj.u_attitude(1) = obj.m * obj.g - ((obj.kP_zdot * obj.zdot_err + ...
                        obj.kI_zdot * obj.zdot_err_sum + ...
                        obj.kD_zdot * (obj.zdot_err - obj.zdot_err_prev)/obj.dt));           
            obj.zdot_err_sum = obj.zdot_err_sum + obj.zdot_err;
            obj.zdot_err_prev = obj.zdot_err;
            
            obj.T = obj.u_attitude(1);
            obj.M = obj.u_attitude(2:4);
        end
        

        function obj = PositionCtrl(obj, position_cmd)
            obj.x_des = position_cmd(1);
            obj.y_des = position_cmd(2);
            obj.z_des = position_cmd(3);
            
            %% Errors in Inertial frame
            obj.x_err = obj.x_des - obj.r(1);
            obj.y_err = obj.y_des - obj.r(2);
            obj.z_err = obj.z_des - obj.r(3);
            
            %% Errors in Body frame
            xy = RPY2Rot([0, 0, obj.euler(3)])*[obj.x_err obj.y_err 1]';
            obj.x_err_b = xy(1);
            obj.y_err_b = xy(2);
            
            %% PIDs
            obj.u_position(1) = -(obj.kP_x * obj.x_err_b + ...
                                 obj.kI_x * obj.x_err_sum + ...
                                 obj.kD_x * (obj.x_err - obj.x_err_prev)/obj.dt);
            obj.u_position(2) = (obj.kP_y * obj.y_err_b + ...
                                 obj.kI_y * obj.y_err_sum + ...
                                 obj.kD_y * (obj.y_err - obj.y_err_prev)/obj.dt);
            obj.u_position(3) = (obj.kP_z * obj.z_err + ...
                                 obj.kI_z * obj.z_err_sum + ...
                                 obj.kD_z * (obj.z_err - obj.z_err_prev)/obj.dt);
            
            obj.x_err_sum = obj.x_err_sum + obj.x_err_b;
            obj.y_err_sum = obj.y_err_sum + obj.y_err_b;
            obj.z_err_sum = obj.z_err_sum + obj.z_err;
            
            obj.x_err_prev = obj.x_err_b;
            obj.y_err_prev = obj.y_err_b;
            obj.z_err_prev = obj.z_err;
            
            obj.attitude_cmd(4) = obj.u_position(3);                        % z_dot_cmd
            obj.attitude_cmd(3) = min(max(position_cmd(4),-pi/6),pi/6);     % psi_cmd
            obj.attitude_cmd(2) = min(max(obj.u_position(1),-pi/6),pi/6);   % theta_cmd
            obj.attitude_cmd(1) = min(max(obj.u_position(2),-pi/6),pi/6);   % phi_cmd
            
        end

    end
end