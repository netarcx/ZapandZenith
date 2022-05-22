package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IMotorSensor;
import com.team1816.lib.math.PoseUtil;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

@Singleton
public class ZenithArm extends Subsystem implements PidProvider {

    public static final double TURRET_JOG_SPEED = 0.15;
    public static final double CARDINAL_SOUTH = 0; // deg
    public static final double CARDINAL_EAST = 270; // deg
    public static final double CARDINAL_NORTH = 180; // deg
    public static final double CARDINAL_WEST = 90; // deg
    public static final String NAME = "arm";
    private static int HALF_ABS_ENCPPR;
    public static int TURRET_LIMIT_REVERSE =
        ((int) factory.getConstant(NAME, "revLimit"));
    public static int TURRET_LIMIT_FORWARD =
        ((int) factory.getConstant(NAME, "fwdLimit"));
    public final int ABS_TICKS_SOUTH;
    public static int ZERO_OFFSET; //used to make sure arm tick range is non-negative

    // Constants
    private static final int kPrimaryCloseLoop = 0;
    private static final int kPIDGyroIDx = 0;
    private static final int kPIDVisionIDx = 0;
    public static int TURRET_ABS_ENCODER_PPR = 4096;
    public static int TURRET_PPR;
    private final int TURRET_MASK;
    private final double TURRET_ENC_RATIO;
    public final int ALLOWABLE_ERROR_TICKS;

    // Components
    private final IMotorControllerEnhanced arm;

    @Inject
    private static Camera camera;

    @Inject
    private static LedManager led;

    private final String pidSlot = "slot0";
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;
    // State
    private static int desiredZenithArmPos = 0;
    private static int followingZenithArmPos = 0;
    private int visionCorroboration = 0;
    private double armSpeed;
    private boolean outputsChanged = true;
    private static ControlMode controlMode;

    public ZenithArm() {
        super(NAME);
        this.arm = factory.getMotor(NAME, "arm");
        TURRET_ABS_ENCODER_PPR = (int) factory.getConstant(NAME, "encPPR");
        TURRET_PPR = (int) factory.getConstant(NAME, "armPPR");
        TURRET_MASK = TURRET_PPR - 1;
        TURRET_ENC_RATIO = (double) TURRET_PPR / TURRET_ABS_ENCODER_PPR;
        ABS_TICKS_SOUTH = ((int) factory.getConstant(NAME, "absPosTicksSouth"));
        HALF_ABS_ENCPPR = TURRET_ABS_ENCODER_PPR / 2 - HALF_ABS_ENCPPR;
        ZERO_OFFSET = (int) factory.getConstant(NAME, "zeroOffset"); //add offset to keep arm in positive range
        arm.setNeutralMode(NeutralMode.Brake);

        PIDSlotConfiguration pidConfig = factory.getPidSlotConfig(NAME, pidSlot);
        this.kP = pidConfig.kP;
        this.kI = pidConfig.kI;
        this.kD = pidConfig.kD;
        this.kF = pidConfig.kF;
        ALLOWABLE_ERROR_TICKS = pidConfig.allowableError.intValue();
        // Position Control
        double peakOutput = 0.75;

        arm.configPeakOutputForward(peakOutput, Constants.kCANTimeoutMs);
        arm.configNominalOutputForward(0, Constants.kCANTimeoutMs);
        arm.configNominalOutputReverse(0, Constants.kCANTimeoutMs);
        arm.configPeakOutputReverse(-peakOutput, Constants.kCANTimeoutMs);
        arm.configAllowableClosedloopError(
            kPIDGyroIDx,
            ALLOWABLE_ERROR_TICKS,
            Constants.kCANTimeoutMs
        );
        arm.configAllowableClosedloopError(
            kPIDVisionIDx,
            ALLOWABLE_ERROR_TICKS,
            Constants.kCANTimeoutMs
        );

        // Soft Limits
        arm.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);
        arm.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);
        arm.configForwardSoftLimitThreshold(
            TURRET_LIMIT_FORWARD,
            Constants.kCANTimeoutMs
        ); // Forward = MAX
        arm.configReverseSoftLimitThreshold(
            TURRET_LIMIT_REVERSE,
            Constants.kCANTimeoutMs
        ); // Reverse = MIN
        arm.overrideLimitSwitchesEnable(true);
        arm.overrideSoftLimitsEnable(true);
    }

    /**
     * converts 0-360 to 0-TURRET_ENCODER_PPR with zero offset
     */
    public static int convertZenithArmDegreesToTicks(double degrees) {
        return ((int) (((degrees) / 360.0) * TURRET_PPR));
    }

    /**
     * converts 0-TURRET_ENCODER_PPR with zero offset
     */
    public double convertZenithArmTicksToDegrees(double ticks) {
        return ticks / TURRET_PPR * 360;
    }

    @Override
    public synchronized void zeroSensors() {
        if (arm instanceof IMotorSensor) {
            var sensors = (IMotorSensor) arm;
            var absSensorVal = sensors.getPulseWidthPosition(); // absolute
            var offset = ZERO_OFFSET - absSensorVal + HALF_ABS_ENCPPR;

            // It is safe to reset quadrature if arm enc reads ~0 (on startup)
            if (
                Math.abs(sensors.getQuadraturePosition()) < HALF_ABS_ENCPPR ||
                    (int) TURRET_ENC_RATIO == 1
            ) {
                //second check - don't zero if abs enc not in viable range
                if (absSensorVal > -1 && absSensorVal < TURRET_ABS_ENCODER_PPR) {
                    sensors.setQuadraturePosition(offset);
                    System.out.println("Zeroing arm limits! Offset: " + offset);
                } else {
                    DriverStation.reportError(
                        "ABSOLUTE ENCODER INVALID RANGE - not zeroing",
                        false
                    );
                }
            } else {
                System.out.println("UNSAFE - NOT ZEROING TURRET QUADRATURE");
            }
        }
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    public void setControlMode(ControlMode controlMode) {
        if (this.controlMode != controlMode) {
            outputsChanged = true;
            if (controlMode == ControlMode.CAMERA_FOLLOWING) {
                if (Constants.kUseVision) {
                    this.controlMode = controlMode;
                    arm.selectProfileSlot(kPIDVisionIDx, 0);
                    camera.setCameraEnabled(true);
                    led.indicateStatus(LedManager.RobotStatus.SEEN_TARGET);
                } else {
                    System.out.println("auto aim not enabled! Not aiming with camera!");
                }
            } else {
                this.controlMode = controlMode;
                arm.selectProfileSlot(kPIDGyroIDx, 0);
                camera.setCameraEnabled(false);
                if (controlMode == ControlMode.MANUAL) {
                    led.indicateStatus(LedManager.RobotStatus.MANUAL_TURRET);
                } else {
                    led.indicateDefaultStatus();
                }
            }
            System.out.println("TURRET CONTROL MODE IS . . . . . . " + this.controlMode);
        }
    }

    @Override
    public double getKP() {
        return kP;
    }

    @Override
    public double getKI() {
        return kI;
    }

    @Override
    public double getKD() {
        return kD;
    }

    @Override
    public double getKF() {
        return kF;
    }

    public void setZenithArmSpeed(double speed) {
        setControlMode(ControlMode.MANUAL);
        if (armSpeed != speed) {
            armSpeed = speed;
            outputsChanged = true;
        }
    }

    private synchronized void setZenithArmPosition(double position) {
        //Since we are using position we need ensure value stays in one rotation
        if (desiredZenithArmPos != (int) position) {
            System.out.println("setting desiredZenithArmPos to " + position);
            desiredZenithArmPos = (int) position;
            outputsChanged = true;
        }
    }

    // CCW positive - 0 to 360
    public synchronized void setZenithArmAngle(double angle) {
        setControlMode(ControlMode.POSITION);
        setZenithArmPosition(convertZenithArmDegreesToTicks(angle));
        followingZenithArmPos = desiredZenithArmPos;
    }

    public synchronized void setFollowingAngle(double angle) {
        if (angle < 0) {
            angle = angle + 360; // if angle is negative, wrap around - we only deal with values from 0 to 360
        }
        setZenithArmPosition(convertZenithArmDegreesToTicks(angle));
    }

    public synchronized void lockZenithArm() {
        setZenithArmAngle(getActualZenithArmPositionDegrees());
    }

    public double getActualZenithArmPositionDegrees() {
        return convertZenithArmTicksToDegrees(getActualZenithArmPositionTicks());
    }

    // this is what is eventually referred to in readFromHardware, so we're undoing conversions here
    public double getActualZenithArmPositionTicks() {
        return (
            (
                arm.getSelectedSensorPosition(kPrimaryCloseLoop) -
                    ABS_TICKS_SOUTH -
                    ZERO_OFFSET
            )
        );
    }

    public double getTargetPosition() {
        if (controlMode == ControlMode.POSITION) {
            return desiredZenithArmPos;
        }
        return followingZenithArmPos;
    }

    public double getPositionError() {
        return getTargetPosition() - getActualZenithArmPositionTicks();
    }

    @Override
    public void readFromHardware() {
        desiredZenithArmPos %= TURRET_PPR;
        followingZenithArmPos %= TURRET_PPR;

        robotState.vehicle_to_arm =
            Rotation2d.fromDegrees(getActualZenithArmPositionDegrees());
    }

    @Override
    public void writeToHardware() {
        switch (controlMode) {
            case CAMERA_FOLLOWING:
                autoHome();
                positionControl(followingZenithArmPos);
                break;
            case FIELD_FOLLOWING:
                trackGyro();
                positionControl(followingZenithArmPos);
                break;
            case CENTER_FOLLOWING:
                trackCenter();
                positionControl(followingZenithArmPos);
                break;
            case ABSOLUTE_FOLLOWING:
                trackAbsolute();
                positionControl(followingZenithArmPos);
                break;
            case ABSOLUTE_MADNESS:
                autoHomeWithOffset(motionOffset());
                positionControl(followingZenithArmPos);
                break;
            case POSITION:
                positionControl(desiredZenithArmPos);
                break;
            case MANUAL:
                manualControl();
                break;
        }
    }

    private int cameraFollowingOffset() {
        var delta = -camera.getDeltaX();
        return ((int) (delta * 26)) - ABS_TICKS_SOUTH;
    }

    private int fieldFollowingOffset() {
        return -convertZenithArmDegreesToTicks( // currently negated because motor is running counterclockwise
            robotState.field_to_vehicle.getRotation().getDegrees()
        );
    }

    private int centerFollowingOffset() {
        double opposite =
            Constants.fieldCenterY - robotState.getFieldToZenithArmPos().getY();
        double adjacent =
            Constants.fieldCenterX - robotState.getFieldToZenithArmPos().getX();
        double armAngle = 0;
        armAngle = Math.atan(opposite / adjacent);
        if (adjacent < 0) armAngle += Math.PI;
        return convertZenithArmDegreesToTicks(Units.radiansToDegrees(armAngle));
    }

    private int motionOffset() {
        Translation2d shooterAxis = new Translation2d(
            robotState.shooterSpeed,
            robotState.getLatestFieldToZenithArm()
        );
        Translation2d driveAxis = new Translation2d(
            robotState.chassis_speeds.vxMetersPerSecond,
            robotState.chassis_speeds.vyMetersPerSecond
        );
        Translation2d predictedTrajectory = driveAxis.unaryMinus().plus(shooterAxis);
        double motionOffsetAngle = PoseUtil.getAngleBetween(
            predictedTrajectory,
            shooterAxis
        );

        if (motionOffsetAngle > Math.PI) {
            motionOffsetAngle -= Math.PI * 2;
        }
        return convertZenithArmDegreesToTicks(Units.radiansToDegrees(motionOffsetAngle));
    }

    private void autoHome() {
        var cameraOffset = cameraFollowingOffset();
        if (cameraOffset > TURRET_PPR / 3) {
            cameraOffset = 0;
        }
        int adj = followingZenithArmPos + cameraOffset;
        //        if (adj > TURRET_LIMIT_FORWARD - ZERO_OFFSET) {
        //            adj = TURRET_LIMIT_FORWARD - ZERO_OFFSET;
        //        } else if (adj < TURRET_LIMIT_REVERSE - ZERO_OFFSET) {
        //            adj = TURRET_LIMIT_REVERSE - ZERO_OFFSET;
        //        }
        if (adj != followingZenithArmPos) {
            followingZenithArmPos = adj;
            outputsChanged = true;
        }
    }

    private void autoHomeWithOffset(int offset) {
        var cameraOffset = cameraFollowingOffset();
        int adj = followingZenithArmPos + cameraOffset + offset;
        if (adj != followingZenithArmPos) {
            followingZenithArmPos = adj;
            outputsChanged = true;
        }
    }

    private void trackGyro() {
        int fieldTickOffset = fieldFollowingOffset();
        int adj = (desiredZenithArmPos + fieldTickOffset);
        if (adj != followingZenithArmPos) {
            followingZenithArmPos = adj;
            outputsChanged = true;
        }
    }

    private void trackCenter() {
        int fieldTickOffset = fieldFollowingOffset();
        int centerOffset = centerFollowingOffset();

        int adj =
            (desiredZenithArmPos + fieldTickOffset + centerOffset + visionCorroboration);
        if (adj != followingZenithArmPos) {
            followingZenithArmPos = adj;
            outputsChanged = true;
        }
    }

    private void trackAbsolute() {
        int fieldTickOffset = fieldFollowingOffset();
        int centerOffset = centerFollowingOffset();
        int motionOffset = motionOffset();

        int adj = (desiredZenithArmPos + fieldTickOffset + centerOffset + motionOffset);
        if (adj != followingZenithArmPos) {
            followingZenithArmPos = adj;
            outputsChanged = true;
        }
    }

    private void positionControl(int rawPos) {
        int adjPos = (rawPos + ABS_TICKS_SOUTH + ZERO_OFFSET) % TURRET_MASK;
        if (adjPos < 0) {
            adjPos += TURRET_MASK;
        }
        if (outputsChanged) {
            arm.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, adjPos);
            outputsChanged = false;
        }
    }

    private void manualControl() {
        if (outputsChanged) {
            //            if (armSpeed == 0) {
            //                arm.set(
            //                    com.ctre.phoenix.motorcontrol.ControlMode.Position,
            //                    getActualZenithArmPositionTicks() + 200 * arm.getMotorOutputPercent()
            //                );
            //            } else {
            arm.set(
                com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput,
                armSpeed
            );
            //            }
            outputsChanged = false;
        }
    }

    @Override
    public void stop() {
        camera.setCameraEnabled(false);
    }

    @Override
    public boolean checkSystem() {
        boolean passed;
        arm.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, .2);
        Timer.delay(2);
        var ticks = getActualZenithArmPositionTicks();
        var diff = Math.abs(ticks - TURRET_LIMIT_FORWARD);
        System.out.println(" + TICKS: " + ticks + "  ERROR: " + diff);
        passed = diff <= 50;
        arm.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, -.2);
        Timer.delay(2);
        ticks = getActualZenithArmPositionTicks();
        diff = Math.abs(ticks - TURRET_LIMIT_REVERSE);
        System.out.println(" - TICKS: " + ticks + "  ERROR: " + diff);
        passed = passed & diff <= 50;
        arm.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
        return passed;
    }

    public enum ControlMode {
        CAMERA_FOLLOWING,
        FIELD_FOLLOWING,
        CENTER_FOLLOWING,
        ABSOLUTE_FOLLOWING,
        ABSOLUTE_MADNESS,
        POSITION,
        MANUAL,

    }
}
