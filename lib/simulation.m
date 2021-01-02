function sim_data = simulation(iterNum,drone,commandSig)
    sim_data = zeros(12, iterNum);
    for i = 1:iterNum
        %drone.PositionCtrl(commandSig);
        drone.AttitudeCtrl(commandSig);
        drone.UpdateState();
        sim_data(:,i) = drone.GetState();
    end
end