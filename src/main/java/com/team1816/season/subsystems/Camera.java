package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.vision.VisionSocket;
import com.team1816.season.proto.VisionProto;
import edu.wpi.first.wpilibj.Timer;

@Singleton
public class Camera extends Subsystem {

    private static final String NAME = "camera";

    public final VisionSocket socket = new VisionSocket();

    // Components
    @Inject
    static LedManager led;

    // Constants
    // private static final double CAMERA_FOV = 87.0; // deg
    private static final double CAMERA_FOCAL_LENGTH = 350; // px
    private static final double VIDEO_WIDTH = 672.0; // px
    public static final double ALLOWABLE_AIM_ERROR = 0.2; // deg

    private double roundTripTime;

    public Camera() {
        super(NAME);
        socket.setDebug(factory.getConstant(NAME, "debug") > 0);
    }

    public double getDeltaXAngle() {
        VisionProto.VisionFrame frame = socket.request("center_x");

        double x = frame.getCenterX();
        if (x < 0) {
            // Reset deltaX to 0 if contour not detected
            return 0;
        }
        double deltaXPixels = (x - (VIDEO_WIDTH / 2)); // Calculate deltaX from center of screen
        return Math.toDegrees(Math.atan2(deltaXPixels, CAMERA_FOCAL_LENGTH)) * 0.64;
    }

    public double getDistance() {
        var timeBefore = Timer.getFPGATimestamp();
        VisionProto.VisionFrame frame = socket.request("distance");
//            System.out.println("CAMERA: getDistance() " + frame.getDistance());
        var timeAfter = Timer.getFPGATimestamp();
        roundTripTime = timeAfter - timeBefore;
        System.out.println("roundTripTime =" + roundTripTime);
        return frame.getDistance();
    }

    public double getRawCenterX() {
        VisionProto.VisionFrame frame = socket.request("center_x");
        return frame.getCenterX();
    }

    public void setEnabled(boolean enabled) {
        led.setCameraLed(enabled);
        socket.setEnabled(enabled);
    }

    public boolean checkSystem() {
        return true;
    }

    public void stop() {
        socket.close();
    }

    public void readFromHardware() {
        if (socket.shouldReconnect()) {
            socket.connect();
        }
    }

    public void simulationPeriodic() {
        double deltaX = this.getDeltaXAngle();
        double distance = this.getDistance();

        System.out.println("{ dx = " + deltaX + ", dist = " + getDistance() + " }");
    }

    public double getRoundTripTime() {
        return roundTripTime;
    }
}
