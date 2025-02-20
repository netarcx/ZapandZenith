package com.team254.lib.util;

import com.team1816.season.Constants;
import com.team1816.lib.math.SwerveKinematics;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Class based on Team 1323's sendInput method to make driving feel better
 */
public class SwerveDriveHelper implements DriveHelper {

    private final static double kHighAdjustmentPower = 1.50; // 1.75 + 0.4375
    private final static double kLowAdjustmentPower = 1.50;
    private final static double kMaxSpeed = (Constants.kOpenLoopMaxVelMeters);
    private final static double kMaxRotation = Constants.kMaxAngularSpeed;

    private final static double kHighPowerRotationScalar = 0.8;
    private final static double kLowPowerRotationScalar = 0.025; //yml time
    private final static double kLowPowerScalar = 0.075; //yml time
    private final static double kRotationExponent = 4.0;
    private final static double kPoleThreshold = 0.0;
    private final static double kRobotRelativePoleThreshold = Math.toRadians(5);
    private final static double kDeadband = 0.15;
    private final static double kRotationDeadband = 0.15;


    public SwerveDriveHelper() {

    }

    @Override
    public SwerveDriveSignal calculateDriveSignal(double forwardInput, double strafeInput, double rotationInput,
                                                  boolean low_power, boolean field_relative, boolean use_heading_controller) {

        Translation2d translationalInput = new Translation2d(forwardInput, strafeInput);
        double inputMagnitude = translationalInput.getNorm();

        // Snap the translational input to its nearest pole, if it is within a certain
        // threshold of it.


        Rotation2d translationalInputDirection = new Rotation2d(translationalInput.getX(), translationalInput.getY());

        if (field_relative) {
            if (Math.abs(translationalInputDirection
                .unaryMinus().rotateBy(nearestPole(translationalInputDirection)).getRadians()) < kPoleThreshold) {
                translationalInput = new Translation2d(nearestPole(translationalInputDirection).getCos(), nearestPole(translationalInputDirection).getSin()).times(inputMagnitude);
            }
        } else {
            if (Math.abs(translationalInputDirection
                .unaryMinus().rotateBy(nearestPole(translationalInputDirection)).getRadians()) < kRobotRelativePoleThreshold) {
                translationalInput = new Translation2d(nearestPole(translationalInputDirection).getCos(), nearestPole(translationalInputDirection).getSin()).times(inputMagnitude);
            }
        }


        if (inputMagnitude < kDeadband) {
            translationalInput = new Translation2d();
            inputMagnitude = 0;
        }

        // Scale x and y by applying a power to the magnitude of the vector they create,
        // in order to make the controls less sensitive at the lower end.
        final double power = (low_power) ? kHighAdjustmentPower : kLowAdjustmentPower;
        Rotation2d direction = translationalInputDirection;
        double scaledMagnitude = Math.pow(inputMagnitude, power);
        translationalInput = new Translation2d(direction.getCos() * scaledMagnitude, direction.getSin() * scaledMagnitude);

        rotationInput = (Math.abs(rotationInput) < kRotationDeadband) ? 0 : rotationInput;
        if (use_heading_controller) { // current constants are tuned to be put to the power of 1.75, and I don't want to retune right now
            rotationInput = Math.pow(Math.abs(rotationInput), 1.75) * Math.signum(rotationInput);
        } else {
            rotationInput = Math.pow(Math.abs(rotationInput), kRotationExponent) * Math.signum(rotationInput);
        }

        translationalInput = translationalInput.times(kMaxSpeed);
        rotationInput *= kMaxRotation;

        if (low_power) {
            translationalInput = translationalInput.times(kLowPowerScalar);
            rotationInput *= kLowPowerRotationScalar;
        } else {
            rotationInput *= kHighPowerRotationScalar;
        }

        return SwerveKinematics.inverseKinematics(translationalInput.getX(), translationalInput.getY(), rotationInput,
            field_relative);
    }

    public Rotation2d nearestPole(Rotation2d rotation) {
        double pole_sin = 0.0;
        double pole_cos = 0.0;
        if (Math.abs(rotation.getCos()) > Math.abs(rotation.getSin())) {
            pole_cos = Math.signum(rotation.getCos());
            pole_sin = 0.0;
        } else {
            pole_cos = 0.0;
            pole_sin = Math.signum(rotation.getSin());
        }
        return new Rotation2d(pole_cos, pole_sin);
    }
}
