package com.team1816.season;

import static com.team1816.season.controlboard.ControlUtils.*;

import badlog.lib.BadLog;
import com.google.inject.Guice;
import com.google.inject.Injector;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.LibModule;
import com.team1816.lib.auto.AutoModeExecutor;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.loops.Looper;
import com.team1816.lib.subsystems.Drive;
import com.team1816.lib.subsystems.DrivetrainLogger;
import com.team1816.lib.subsystems.SubsystemManager;
import com.team1816.season.auto.AutoModeSelector;
import com.team1816.season.auto.paths.TrajectorySet;
import com.team1816.season.controlboard.ActionManager;
import com.team1816.season.states.RobotState;
import com.team1816.season.states.Superstructure;
import com.team1816.season.subsystems.*;
import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.util.SwerveDriveSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Optional;

public class Robot extends TimedRobot {

    private BadLog logger;

    private final Injector injector;

    private final Looper mEnabledLooper = new Looper(this);
    private final Looper mDisabledLooper = new Looper(this);

    private IControlBoard mControlBoard;

    private final SubsystemManager mSubsystemManager;

    //State managers
    private final Superstructure mSuperstructure;
    private final Infrastructure mInfrastructure;
    private final com.team1816.season.states.RobotState mRobotState;

    // subsystems
    private final Drive mDrive;
    private final Collector mCollector;
    private final Shooter mShooter;
    private final Turret mTurret;
    private final Spindexer mSpindexer;
    private final Elevator mElevator;
    private final Climber mClimber;
    private final Camera mCamera;
    private final LedManager mLedManager;
    private final DistanceManager mDistanceManager;

    private final LatchedBoolean mWantsAutoExecution = new LatchedBoolean();
    private final LatchedBoolean mWantsAutoInterrupt = new LatchedBoolean();

    private final AutoModeSelector mAutoModeSelector;
    private final AutoModeExecutor mAutoModeExecutor;
    private TrajectorySet trajectorySet;

    private double loopStart;

    private ActionManager actionManager;

    // private PowerDistributionPanel pdp = new PowerDistributionPanel();
    private final Turret.ControlMode defaultTurretControlMode =
        Turret.ControlMode.FIELD_FOLLOWING;
    private boolean faulted;
    private boolean useManualShoot = false;

    Robot() {
        super();
        // initialize injector
        injector = Guice.createInjector(new LibModule(), new SeasonModule());
        mDrive = (injector.getInstance(Drive.Factory.class)).getInstance();
        mTurret = injector.getInstance(Turret.class);
        mClimber = injector.getInstance(Climber.class);
        mCollector = injector.getInstance(Collector.class);
        mElevator = injector.getInstance(Elevator.class);
        mCamera = injector.getInstance(Camera.class);
        mSpindexer = injector.getInstance(Spindexer.class);
        mSuperstructure = injector.getInstance(Superstructure.class);
        mShooter = injector.getInstance(Shooter.class);
        mRobotState = injector.getInstance(RobotState.class);
        mInfrastructure = injector.getInstance(Infrastructure.class);
        mDistanceManager = injector.getInstance(DistanceManager.class);
        mLedManager = injector.getInstance(LedManager.class);
        mSubsystemManager = injector.getInstance(SubsystemManager.class);
        mAutoModeSelector = injector.getInstance(AutoModeSelector.class);
        mAutoModeExecutor = injector.getInstance(AutoModeExecutor.class);
        trajectorySet = injector.getInstance(TrajectorySet.class);
    }

    public static RobotFactory getFactory() {
        return RobotFactory.getInstance();
    }

    private Double getLastLoop() {
        return (Timer.getFPGATimestamp() - loopStart) * 1000;
    }

    @Override
    public void robotInit() {
        try {
            mControlBoard = injector.getInstance(IControlBoard.class);
            DriverStation.silenceJoystickConnectionWarning(true);
            if (Constants.kIsBadlogEnabled) {
                var logFile = new SimpleDateFormat("MMdd_HH-mm").format(new Date());
                var robotName = System.getenv("ROBOT_NAME");
                if (robotName == null) robotName = "default";
                var logFileDir = "/home/lvuser/";
                // if there is a usb drive use it
                if (Files.exists(Path.of("/media/sda1"))) {
                    logFileDir = "/media/sda1/";
                }
                if (RobotBase.isSimulation()) {
                    if (System.getProperty("os.name").toLowerCase().contains("win")) {
                        logFileDir = System.getenv("temp") + "\\";
                    } else {
                        logFileDir = System.getProperty("user.dir") + "/";
                    }
                }
                var filePath = logFileDir + robotName + "_" + logFile + ".bag";
                logger = BadLog.init(filePath);

                BadLog.createValue(
                    "Max Velocity",
                    String.valueOf(Constants.kPathFollowingMaxVelMeters)
                );
                BadLog.createValue(
                    "Max Acceleration",
                    String.valueOf(Constants.kPathFollowingMaxAccel)
                );

                BadLog.createTopic(
                    "Timings/Looper",
                    "ms",
                    mEnabledLooper::getLastLoop,
                    "hide",
                    "join:Timings"
                );
                BadLog.createTopic(
                    "Timings/RobotLoop",
                    "ms",
                    this::getLastLoop,
                    "hide",
                    "join:Timings"
                );
                BadLog.createTopic(
                    "Timings/Timestamp",
                    "s",
                    Timer::getFPGATimestamp,
                    "xaxis",
                    "hide"
                );

                DrivetrainLogger.init(mDrive);
                if (RobotBase.isReal()) {
                    BadLog.createTopic(
                        "PDP/Current",
                        "Amps",
                        mInfrastructure.getPdh()::getTotalCurrent
                    );

                    mDrive.CreateBadLogValue("Drivetrain PID", mDrive.pidToString());
                    mTurret.CreateBadLogValue("Turret PID", mTurret.pidToString());
                    mShooter.CreateBadLogValue("Shooter PID", mShooter.pidToString());

                    if (mCamera.isImplemented()) {
                        BadLog.createTopic(
                            "Vision/DeltaXAngle",
                            "Degrees",
                            mCamera::getDeltaX
                        );
                        BadLog.createTopic(
                            "Vision/Distance",
                            "inches",
                            mCamera::getDistance
                        );
                    }
                }
                mShooter.CreateBadLogTopic(
                    "Shooter/ActVel",
                    "NativeUnits",
                    mShooter::getActualVelocity,
                    "hide",
                    "join:Shooter/Velocities"
                );
                mShooter.CreateBadLogTopic(
                    "Shooter/TargetVel",
                    "NativeUnits",
                    mShooter::getTargetVelocity,
                    "hide",
                    "join:Shooter/Velocities"
                );
                mShooter.CreateBadLogTopic(
                    "Shooter/Error",
                    "NativeUnits",
                    mShooter::getError,
                    "hide",
                    "join:Shooter/Velocities"
                );
                mTurret.CreateBadLogTopic(
                    "Turret/ActPos",
                    "NativeUnits",
                    mTurret::getActualTurretPositionTicks,
                    "hide",
                    "join:Turret/Positions"
                );
                mTurret.CreateBadLogTopic(
                    "Turret/TargetPos",
                    "NativeUnits",
                    mTurret::getTargetPosition,
                    "hide",
                    "join:Turret/Positions"
                );
                mTurret.CreateBadLogTopic(
                    "Turret/ErrorPos",
                    "NativeUnits",
                    mTurret::getPositionError
                );
            }

            logger.finishInitialization();

            mSubsystemManager.setSubsystems(
                mDrive,
                mElevator,
                mSpindexer,
                mShooter,
                mCollector,
                mTurret,
                mClimber,
                mCamera,
                mLedManager
            );

            mSubsystemManager.zeroSensors();
            mSuperstructure.setStopped(true); // bool statement is for shooter state (stop or coast)
            mDistanceManager.outputBucketOffsets();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            // Robot starts forwards.
            mRobotState.reset();

            mAutoModeSelector.updateModeCreator();

            actionManager =
                new ActionManager(
                    // Driver Gamepad
                    //                    createAction(
                    //                        mControlBoard::getRunAutoModeInTeleop,
                    //                        () -> {
                    //                            System.out.println("Running trajectory !");
                    //                            SmartDashboard.putString("Teleop Spline", "TWO_BALL_B");
                    //                            var trajectory = new TrajectoryAction(
                    //                                TrajectorySet.TWO_BALL_B,
                    //                                TrajectorySet.TWO_BALL_B_HEADINGS
                    //                            );
                    //                            mDrive.zeroSensors(
                    //                                trajectory.getTrajectory().getInitialPose()
                    //                            );
                    //                            trajectory.start();
                    //                        }
                    //                    ),
                    createHoldAction(
                        mControlBoard::getCollectorToggle,
                        pressed -> mSuperstructure.setCollecting(pressed, true)
                    ),
                    createHoldAction(
                        mControlBoard::getCollectorBackspin,
                        pressed -> mSuperstructure.setCollecting(pressed, false)
                    ),
                    createAction(mControlBoard::getUnlockClimber, mClimber::unlock),
                    createAction(
                        mControlBoard::getUseManualShoot,
                        () -> {
                            useManualShoot = !useManualShoot;
                            System.out.println("manual shooting toggled!");
                        }
                    ),
                    createAction(
                        mControlBoard::getZeroPose,
                        () -> {
                            mTurret.setTurretAngle(Turret.CARDINAL_SOUTH);
                            mDrive.zeroSensors(Constants.ZeroPose);
                        }
                    ),
                    createHoldAction(mControlBoard::getSlowMode, mDrive::setSlowMode),
                    // Operator Gamepad
                    createAction(
                        mControlBoard::getRaiseBucket,
                        () -> mDistanceManager.incrementBucket(100)
                    ),
                    createAction(
                        mControlBoard::getLowerBucket,
                        () -> mDistanceManager.incrementBucket(-100)
                    ),
                    createAction(
                        mControlBoard::getAutoAim,
                        () -> {
                            mTurret.snapWithCamera();
                        }
                    ),
                    createAction(mControlBoard::getCameraToggle, mCamera::toggleEnabled),
                    createHoldAction(
                        mControlBoard::getYeetShot,
                        yeet -> {
                            mShooter.setHood(false);
                            if (useManualShoot) {
                                mSuperstructure.setRevving(
                                    yeet,
                                    Shooter.TARMAC_TAPE_VEL,
                                    true
                                ); // Tarmac
                            } else {
                                mSuperstructure.setRevving(
                                    yeet,
                                    Shooter.NEAR_VELOCITY,
                                    true
                                ); // Barf shot
                            }
                            mSuperstructure.setFiring(yeet);
                        }
                    ),
                    createHoldAction(
                        mControlBoard::getShoot,
                        shooting -> {
                            mShooter.setHood(true);
                            mSuperstructure.setRevving(
                                shooting,
                                Shooter.LAUNCHPAD_VEL,
                                useManualShoot // use manual shoot WAS HERE
                            ); // Launchpad
                            mSuperstructure.setFiring(shooting);
                        }
                    ),
                    createAction(mControlBoard::getHood, mShooter::setHood),
                    createHoldAction(
                        mControlBoard::getTurretJogLeft,
                        moving ->
                            mTurret.setTurretSpeed(moving ? Turret.TURRET_JOG_SPEED : 0)
                    ),
                    createHoldAction(
                        mControlBoard::getTurretJogRight,
                        moving ->
                            mTurret.setTurretSpeed(moving ? -Turret.TURRET_JOG_SPEED : 0)
                    ),
                    createHoldAction(
                        mControlBoard::getClimberUp,
                        moving -> mClimber.setClimberPower(moving ? -.5 : 0)
                    ),
                    createHoldAction(
                        mControlBoard::getClimberDown,
                        moving -> mClimber.setClimberPower(moving ? .5 : 0)
                    ),
                    createAction(mControlBoard::getTopClamp, mClimber::setTopClamp),
                    createAction(mControlBoard::getBottomClamp, mClimber::setBottomClamp),
                    createAction(
                        mControlBoard::getAutoClimb,
                        () -> {
                            if (mClimber.getCurrentStage() == 0) {
                                mTurret.setTurretAngle(Turret.CARDINAL_SOUTH);
                                mSuperstructure.setStopped(true);
                            } else {
                                mTurret.setTurretAngle(Turret.CARDINAL_SOUTH - 30);
                            }

                            mClimber.incrementClimberStage();
                        }
                    )
                );
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            mEnabledLooper.stop();

            mLedManager.setDefaultStatus(LedManager.RobotStatus.DISABLED);
            mCamera.setCameraEnabled(false);

            mSuperstructure.setStopped(true);

            // Reset all auto mode state.
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mAutoModeSelector.updateModeCreator();

            mDisabledLooper.start();

            mDrive.stop();
            mDrive.setBrakeMode(false);
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            mDisabledLooper.stop();
            mLedManager.setDefaultStatus(LedManager.RobotStatus.AUTONOMOUS);

            // Robot starts where it's told for auto path
            mRobotState.reset();

            mDrive.zeroSensors();
            mTurret.zeroSensors();

            mSuperstructure.setStopped(false);

            mDrive.setControlState(Drive.DriveControlState.TRAJECTORY_FOLLOWING);
            mAutoModeExecutor.start();

            mEnabledLooper.start();
        } catch (Throwable t) {
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            mDisabledLooper.stop();
            mLedManager.setDefaultStatus(LedManager.RobotStatus.ENABLED);

            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mDrive.stop();
            mTurret.zeroSensors();
            mClimber.zeroSensors();

            mEnabledLooper.start();

            mTurret.setTurretAngle(Turret.CARDINAL_SOUTH);
            mTurret.setControlMode(defaultTurretControlMode);

            mSuperstructure.setStopped(false);
            mInfrastructure.startCompressor();
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    @Override
    public void testInit() {
        try {
            double initTime = System.currentTimeMillis();

            mLedManager.blinkStatus(LedManager.RobotStatus.DRIVETRAIN_FLIPPED);
            // Warning - blocks thread - intended behavior?
            while (System.currentTimeMillis() - initTime <= 3000) {
                mLedManager.writeToHardware();
            }

            mSuperstructure.setStopped(false);

            mEnabledLooper.stop();
            mDisabledLooper.start();
            mTurret.zeroSensors();
            mDrive.zeroSensors();

            mLedManager.blinkStatus(LedManager.RobotStatus.DISABLED);

            if (mSubsystemManager.checkSubsystems()) {
                System.out.println("ALL SYSTEMS PASSED");
                mLedManager.indicateStatus(LedManager.RobotStatus.ENABLED);
            } else {
                System.err.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
                mLedManager.indicateStatus(LedManager.RobotStatus.ERROR);
            }
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            // update shuffleboard for subsystem values
            mSubsystemManager.outputToSmartDashboard();
            // update robot state on field for Field2D widget
            mRobotState.outputToSmartDashboard();
            mAutoModeSelector.outputToSmartDashboard();
        } catch (Throwable t) {
            faulted = true;
            System.out.println(t.getMessage());
        }
    }

    @Override
    public void disabledPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        try {
            if (RobotController.getUserButton()) {
                mDrive.zeroSensors(Constants.ZeroPose);
                mLedManager.indicateStatus(LedManager.RobotStatus.SEEN_TARGET);
            } else {
                // non-camera LEDs will flash red if robot periodic updates fail
                if (faulted) {
                    mLedManager.blinkStatus(LedManager.RobotStatus.ERROR);
                } else {
                    mLedManager.indicateStatus(LedManager.RobotStatus.DISABLED);
                }
            }

            // Periodically check if drivers changed desired auto - if yes, then update the actual auto mode
            mAutoModeSelector.updateModeCreator();

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            if (
                autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()
            ) {
                var auto = autoMode.get();
                System.out.println("Set auto mode to: " + auto.getClass().toString());
                mRobotState.field.getObject("Trajectory");
                mAutoModeExecutor.setAutoMode(auto);
                Constants.StartingPose = auto.getTrajectory().getInitialPose();
            }
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        loopStart = Timer.getFPGATimestamp();

        // debugging functionality to stop autos mid-path
        boolean signalToResume = !mControlBoard.getUnlockClimber();
        boolean signalToStop = mControlBoard.getUnlockClimber();
        if (mAutoModeExecutor.isInterrupted()) {
            manualControl();

            if (mWantsAutoExecution.update(signalToResume)) {
                mAutoModeExecutor.resume();
            }
        }
        if (mWantsAutoInterrupt.update(signalToStop)) {
            mAutoModeExecutor.interrupt();
        }

        if (Constants.kIsLoggingAutonomous) {
            logger.updateTopics();
            logger.log();
        }
    }

    @Override
    public void teleopPeriodic() {
        loopStart = Timer.getFPGATimestamp();

        try {
            manualControl(); // controls drivetrain and turret joystick control mode
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
        if (Constants.kIsLoggingTeleOp) {
            logger.updateTopics();
            logger.log();
        }
    }

    public void manualControl() {
        actionManager.update();

        if (
            Math.abs(mControlBoard.getTurretXVal()) > 0.90 ||
            Math.abs(mControlBoard.getTurretYVal()) > 0.90
        ) {
            mTurret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
            mTurret.setFollowingAngle(
                (
                    new Rotation2d(
                        mControlBoard.getTurretXVal(),
                        mControlBoard.getTurretYVal()
                    )
                ).getDegrees()
            );
        }

        if (mControlBoard.getBrakeMode()) {
            mDrive.setOpenLoop(SwerveDriveSignal.BRAKE);
        } else {
            mDrive.setTeleopInputs(
                mControlBoard.getThrottle(),
                mControlBoard.getStrafe(),
                mControlBoard.getTurn(),
                mControlBoard.getSlowMode(),
                false
            );
        }
    }

    @Override
    public void testPeriodic() {}
}
