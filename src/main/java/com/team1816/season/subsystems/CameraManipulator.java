package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.subsystems.TankDrive;


@Singleton
public class CameraManipulator extends Subsystem {

    private static final String NAME = "cameraManipulator";

    // Constants
    public final int REV_LIMIT = Math.min(
        (int) factory.getConstant(NAME,"fwdLimit"),
        ((int) factory.getConstant(NAME,"revLimit"))
    );
    public final int FWD_LIMIT = Math.max(
        (int) factory.getConstant(NAME, "fwdLimit"),
        ((int) factory.getConstant(NAME, "revLimit"))
    );

    public static int SLIDER_PPR;

    // Components

    private final IGreenMotor cameraSlider;

    @Inject
    private static Turret turret;

    @Inject
    private static LedManager led;

    @Inject
    private static TankDrive drivetrain;

    // State

    private int desiredPos = 0;

    private int followingPos = 0;

    private double sliderSpeed;


    private boolean outputsChanged = true; //TODO: Turret class starts this as true, see if needed. SLider is going to need to calibrate on bootup

    private ControlMode controlMode;

    public CameraManipulator() {
        super(NAME);
        this.cameraSlider = factory.getMotor(NAME,"cameraSlider"); //TODO: Add to YAML
        SLIDER_PPR = (int) factory.getConstant(NAME,"sliderPPR"); //TODO: Add to YAML


    }

    @Override
    public void zeroSensors() {

    }

    public void calibrateSlider() {

    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public enum ControlMode {
        //TODO: Add States needed
        VISION_FOLLOWING,
        FIELD_FOLLOWING,
        POSITION,
        MANUAL,
    }
}
