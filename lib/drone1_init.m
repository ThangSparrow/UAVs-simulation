%%  INIT. PARAMS.
drone1_params = containers.Map({'Mass', 'armLength', 'Ixx', 'Iyy', 'Izz'}, ...
    {1.25, 0.265, 0.0232, 0.0232, 0.0468});

drone1_initStates = [0, 0, -5, ...               % X, Y, Z
                     0, 0, 0, ...               % dX, dY, dZ
                     0, 0, 0, ...               % phi, theta, psi
                     0, 0, 0]';                  % p, q, r

drone1_initInputs = [0, 0, 0, 0]';                  % u1, u2, u3, u4 (T, M1, M2, M3)

% drone1_body = [ 0.265,      0,     0,   1; ...  % first arm
%                     0, -0.265,     0,   1; ...  % second arm
%                -0.265,      0,     0,   1; ...  % third arm
%                     0,  0.265,     0,   1; ...  % fouth arm
%                     0,      0,     0,   1; ...  % center
%                     0,      0, -0.15,   1]';    % controller
                
drone1_gains = containers.Map(...
    {  'P_phi',   'I_phi',   'D_phi', ...
     'P_theta', 'I_theta', 'D_theta', ...
       'P_psi',   'I_psi',   'D_psi', ...
      'P_zdot',  'I_zdot',  'D_zdot', ...
         'P_x',     'I_x',     'D_x',...
         'P_y',     'I_y',     'D_y', ...
         'P_z',     'I_z',     'D_z',}, ...
    {      15,       0.005,       0.9, ...
           10,      0.0,            1, ...
           1,        0.0,           1, ...
           1.0,     0.0,          0.0, ...
           0.1,      0.0,         0.1, ...
           0.1,      0.0,         0.1, ...
           1.0,      0.0,         1.0});
