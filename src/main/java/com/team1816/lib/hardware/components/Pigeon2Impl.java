package com.team1816.lib.hardware.components;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.team1816.season.Constants;

public class Pigeon2Impl extends Pigeon2 implements IPigeonIMU {

    public Pigeon2Impl(int id) {
        super(id, Constants.CANBusHighSpeed);
    }

    // set Fused heading doesn't exist in pigeon 2. De we do compass calibration? If not then fused heading is about the same as yaw.
    @Override
    public double getYaw() {
        return super.getYaw();
    }

    @Override
    public ErrorCode setYaw(double angleDeg) {
        return super.setYaw(angleDeg);
    }

    @Override
    public ErrorCode setFusedHeading(double angleDeg) {
        return super.setYaw(angleDeg);
    }

    @Override
    public ErrorCode setAccumZAngle(double angleDeg) {
        return super.setAccumZAngle(angleDeg);
    }

    @Override
    public boolean hasResetOccurred() {
        return super.hasResetOccurred();
    }

    @Override
    public ErrorCode configFactoryDefault() {
        return super.configFactoryDefault();
    }
}
