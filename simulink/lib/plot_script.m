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
    simPoins = size(data,1);
    subplot(2,3,1)
        plot(1:simPoins,rad2deg(data(:,7)),'r-');
    subplot(2,3,2)
        plot(1:simPoins,rad2deg(data(:,8)),'r-');    
    subplot(2,3,3)
        plot(1:simPoins,rad2deg(data(:,9)),'-');
        plot(1:simPoins,cmdSig(4)*ones(1,simPoins),'b-');
    subplot(2,3,4)
        plot(1:simPoins,data(:,1),'r-');
        plot(1:simPoins,cmdSig(1)*ones(1,simPoins),'b-');
    subplot(2,3,5)
        plot(1:simPoins,data(:,2),'r-');
        plot(1:simPoins,cmdSig(2)*ones(1,simPoins),'b-');
    subplot(2,3,6)
        plot(1:simPoins,data(:,3),'r-');
        plot(1:simPoins,cmdSig(3)*ones(1,simPoins),'b-');
end