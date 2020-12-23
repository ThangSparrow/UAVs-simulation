close all;

%% Init. 3D Fig.
fig1 = figure('pos',[0 90 600 600]);
view(3);
fig1.CurrentAxes.ZDir = 'Reverse';
fig1.CurrentAxes.YDir = 'Reverse';

axis equal;
grid on;

xlim([-5 5]); ylim([-5 5]); zlim([-8 0]);
xlabel('X[m]'); ylabel('Y[m]'); zlabel('Z[m]');

hold(gca, 'on');
drone1_state = drone1.GetState();
wHb = [RPY2Rot(drone1_state(7:9))' drone1_state(1:3); 0 0 0 1];
drone1_world = wHb * drone1_body;
drone1_atti = drone1_world(1:3, :);

fig1_ARM13 = plot3(gca, drone1_atti(1,[1 3]), drone1_atti(2,[1 3]), drone1_atti(3,[1 3]), '-ro', 'MarkerSize', 5);
fig1_ARM24 = plot3(gca, drone1_atti(1,[2 4]), drone1_atti(2,[2 4]), drone1_atti(3,[2 4]), '-bo', 'MarkerSize', 5);
fig1_payload = plot3(gca, drone1_atti(1,[5 6]), drone1_atti(2,[5 6]), drone1_atti(3,[5 6]), '-k', 'Linewidth', 3);
fig1_shadow = plot3(gca, 0, 0, 0, 'xk', 'LineWidth', 3);

hold(gca,'off');

%% Init. Data Fig.
fig2 = figure('pos',[600 290 600 400]);
subplot(2,3,1)
title('phi[deg]');
grid on; hold on;

subplot(2,3,2)
title('theta[deg]');
grid on; hold on;

subplot(2,3,3)
title('psi[deg]');
grid on; hold on;

subplot(2,3,4)
title('x[m]');
grid on; hold on;

subplot(2,3,5)
title('y[m]');
grid on; hold on;

subplot(2,3,6)
title('z[m]');
grid on; hold on;

%% ANIMATION
for i = 1:simTime/0.01
   drone1.PositionCtrl(commandSig);
   drone1.AttitudeCtrl(); 
   drone1.UpdateState();
   
   drone1_state = drone1.GetState();
   
   %% 3D Plot
   figure(1)
   wHb = [RPY2Rot(drone1_state(7:9))' drone1_state(1:3); 0 0 0 1];
   drone1_world = wHb * drone1_body;
   drone1_atti = drone1_world(1:3, :);
   
   set(fig1_ARM13, ...
       'xData', drone1_atti(1,[1 3]), ...
       'yData', drone1_atti(2,[1 3]), ...
       'zData', drone1_atti(3,[1 3]));
   
   set(fig1_ARM24, ...
       'xData', drone1_atti(1,[2 4]), ...
       'yData', drone1_atti(2,[2 4]), ...
       'zData', drone1_atti(3,[2 4]));
   
   set(fig1_payload, ...
       'xData', drone1_atti(1,[5 6]), ...
       'yData', drone1_atti(2,[5 6]), ...
       'zData', drone1_atti(3,[5 6]));
       
   set(fig1_shadow, ...
       'xData', drone1_atti(1,5), ...
       'yData', drone1_atti(2,5), ...
       'zData', 0);
   
   %% Data Plot
    figure(2)
    subplot(2,3,1)
        plot(i/100,rad2deg(drone1_state(7)),'.');
    subplot(2,3,2)
        plot(i/100,rad2deg(drone1_state(8)),'.');    
    subplot(2,3,3)
        plot(i/100,rad2deg(drone1_state(9)),'.');
    subplot(2,3,4)
        plot(i/100,drone1_state(1),'.');
    subplot(2,3,5)
        plot(i/100,drone1_state(2),'.');
    subplot(2,3,6)
        plot(i/100,drone1_state(3),'.');
       
end