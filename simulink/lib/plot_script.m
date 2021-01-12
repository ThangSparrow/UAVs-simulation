function plot_script(data,cmdSig,stepSize)
    %% Init. Data Fig.
    figure('pos',[100 40 1200 650]);
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
    
    %% PLOT DATA
    simTime = size(data,1)*stepSize;
    simPoints = stepSize:stepSize:simTime;
    subplot(2,3,1)
        plot(simPoints,rad2deg(data(:,7)),'r-');
    subplot(2,3,2)
        plot(simPoints,rad2deg(data(:,8)),'r-');    
    subplot(2,3,3)
        plot(simPoints,rad2deg(data(:,9)),'-');
        plot(simPoints,cmdSig(4)*ones(1,size(simPoints,1)),'b-');
    subplot(2,3,4)
        plot(simPoints,data(:,1),'r-');
        plot(simPoints,cmdSig(1)*ones(1,size(simPoints,1)),'b-');
        legend({'Measured','Desired'},'Location','southoutside')
    subplot(2,3,5)
        plot(simPoints,data(:,2),'r-');
        plot(simPoints,cmdSig(2)*ones(1,size(simPoints,1)),'b-');
        legend({'Measured','Desired'},'Location','southoutside')
    subplot(2,3,6)
        plot(simPoints,data(:,3),'r-');
        plot(simPoints,cmdSig(3)*ones(1,size(simPoints,1)),'b-');
        legend({'Measured','Desired'},'Location','southoutside')
end