package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import org.checkerframework.checker.units.qual.C;

public class Climber extends Subsystem {
    // climber this year has 2 motors: one for going up/down, and one for making the ferris wheel "spin". 4? pistons,
    // one for each claw, (two will be fired at once) - If you have no idea what the climber does or looks like,
    // ask max or nico or, even better, bully (ask) the build team because they better know about what they're building!

    private static final String NAME = "climber";

    // Components
    private final CANSparkMax elevator;

    // State
    private double climberPow;
    private boolean isDeployed;
    private boolean outputsChanged = false;

    public Climber() {
        super(NAME);
        elevator =  new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);//factory.getMotor(NAME, "elevator");
    }

    public void setClimberPower(double power) {
        climberPow = power;
        outputsChanged = true;
    }

    public boolean getDeployed() {
        return isDeployed;
    }

    @Override
    public void writePeriodicOutputs() {
        if (outputsChanged) {
//            elevator.set(ControlMode.PercentOutput, climberPow);
            elevator.set(climberPow);
            outputsChanged = false;
        }
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }
}
