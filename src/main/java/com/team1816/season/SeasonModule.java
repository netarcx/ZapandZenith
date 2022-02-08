package com.team1816.season;

import com.google.inject.AbstractModule;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.season.auto.actions.actions2020.*;
import com.team1816.season.controlboard.ControlBoard;
import com.team1816.season.controlboard.GamepadButtonControlBoard;
import com.team1816.season.controlboard.GamepadDriveControlBoard;
import com.team1816.season.paths.TrajectorySet;
import com.team1816.season.subsystems.*;
import com.team1816.lib.auto.AutoModeExecutor;
import com.team1816.lib.controlboard.IButtonControlBoard;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.controlboard.IDriveControlBoard;
import com.team1816.lib.subsystems.Infrastructure;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SeasonModule extends AbstractModule {

    @Override
    protected void configure() {
        bind(Drive.Factory.class).to(SeasonFactory.class);
        bind(IControlBoard.class).to(ControlBoard.class);
        bind(IDriveControlBoard.class).to(GamepadDriveControlBoard.class);
        bind(IButtonControlBoard.class).to(GamepadButtonControlBoard.class);
        requestStaticInjection(DifferentialDriveKinematics.class);
        requestStaticInjection(SwerveDriveKinematics.class);
        requestStaticInjection(Drive.class);
        requestStaticInjection(TankDrive.class);
        requestStaticInjection(SwerveDrive.class);
        requestStaticInjection(Infrastructure.class);
        requestStaticInjection(Camera.class);
        requestStaticInjection(Turret.class);
        requestStaticInjection(AutoModeSelector.class);
        requestStaticInjection(AutoModeExecutor.class);
        requestStaticInjection(DistanceManager.class);
        requestStaticInjection(TrajectorySet.class);
        requestStaticInjection(TrajectoryAction.class);
        requestStaticInjection(TurretAction.class);
    }
}