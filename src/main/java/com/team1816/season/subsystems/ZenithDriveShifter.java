package com.team1816.season.subsystems;

import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;

public class ZenithDriveShifter extends Subsystem {

    private static final String NAME = "driveShifter";

    //Components
    private final ISolenoid shifterSolenoid;
    private double shifterSpread = factory.getConstant(NAME,"shifterSpread",0.5);
    private boolean isHighGear;
    private boolean outputsChanged = false;
    private STATE desiredState = STATE.STOP;


    public ZenithDriveShifter() {
        super(NAME);
        shifterSolenoid = factory.getSolenoid(NAME, "driveShifter");


    }

   public void setDesiredState(STATE state) {
        if(desiredState != state){
            desiredState = state;
            outputsChanged = true;
        }

   }


    @Override
    public void writeToHardware() {
        if(outputsChanged){
            outputsChanged = false;
            switch(desiredState) {
                case STOP:
                    isHighGear =false;
                    break;
                case HIGHGEAR:
                    isHighGear = true;
                    break;
                case LOWGEAR:
                    isHighGear = false;
                    break;
            }
            shifterSolenoid.set(isHighGear);
        }


    }

    @Override
    public void zeroSensors() {
        this.setDesiredState(STATE.STOP); // on boot-up will set the shifter state to stopped
    }


    @Override
    public void stop() {
        setDesiredState(STATE.STOP);

    }

    public void setHighGear() {
        setDesiredState(STATE.HIGHGEAR);
    }

    public void setLowGear() {
        setDesiredState(STATE.LOWGEAR);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public enum STATE {
        STOP,
        HIGHGEAR,
        LOWGEAR,
    }

}
