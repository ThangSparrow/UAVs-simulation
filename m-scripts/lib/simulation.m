function sim_data = simulation(iterNum,drone,commandSig,posCtrl)
    sim_data = zeros(12, iterNum);
    for i = 1:iterNum
        if posCtrl == 1
            drone.PositionCtrl(commandSig);
            drone.AttitudeCtrl([]);
        else
            drone.AttitudeCtrl(commandSig);
        end
        drone.UpdateState();
        sim_data(:,i) = drone.GetState();
    end
end