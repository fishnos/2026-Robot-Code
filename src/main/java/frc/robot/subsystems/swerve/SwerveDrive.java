package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState;
import frc.robot.RobotState.OdometryObservation;
import frc.robot.constants.Constants;
import frc.robot.configs.SwerveConfig;
import frc.robot.configs.SwerveDrivetrainConfig;
import frc.robot.configs.SwerveModuleGeneralConfig;
import frc.robot.lib.util.ConfigLoader;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator;
import frc.robot.lib.BLine.ChassisRateLimiter;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOInputsAutoLogged;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.module.ModuleIOTalonFX;

public class SwerveDrive extends SubsystemBase {
    private static SwerveDrive instance = null;
    public static SwerveDrive getInstance() {
        if (instance == null) {
            instance = new SwerveDrive();
        }

        return instance;
    }

    // FSM State Enums
    public enum DesiredSystemState {
        DISABLED,
        IDLE,
        TELEOP,
        FOLLOW_PATH,
        PREPARE_FOR_AUTO,
        SYSID
    }

    public enum CurrentSystemState {
        DISABLED,
        IDLE,
        TELEOP,
        FOLLOW_PATH,
        PREPARE_FOR_AUTO,
        READY_FOR_AUTO,
        SYSID
    }

    public enum DesiredOmegaOverrideState {
        NONE,
        RANGED_ROTATION,
        SNAPPED,
    }

    public enum CurrentOmegaOverrideState {
        NONE,
        RANGED_NOMINAL,
        RANGED_RETURNING,
        SNAPPED_NOMINAL,
        SNAPPED_RETURNING,
    }

    public enum DesiredTranslationOverrideState {
        NONE,
        FROZEN,
        CAPPED
    }

    public enum CurrentTranslationOverrideState {
        NONE,
        FROZEN,
        CAPPED
    }


    // FSM State Variables
    private DesiredSystemState desiredSystemState = DesiredSystemState.DISABLED;
    private CurrentSystemState currentSystemState = CurrentSystemState.DISABLED;
    private CurrentSystemState previousSystemState = CurrentSystemState.DISABLED;

    private DesiredOmegaOverrideState desiredOmegaOverrideState = DesiredOmegaOverrideState.NONE;
    private CurrentOmegaOverrideState currentOmegaOverrideState = CurrentOmegaOverrideState.NONE;
    private CurrentOmegaOverrideState previousOmegaOverrideState = CurrentOmegaOverrideState.NONE;

    private DesiredTranslationOverrideState desiredTranslationOverrideState = DesiredTranslationOverrideState.NONE;
    private CurrentTranslationOverrideState currentTranslationOverrideState = CurrentTranslationOverrideState.NONE;
    private CurrentTranslationOverrideState previousTranslationOverrideState = CurrentTranslationOverrideState.NONE;

    // Teleop input suppliers (normalized -1 to 1)
    private DoubleSupplier vxNormalizedSupplier = () -> 0.0;
    private DoubleSupplier vyNormalizedSupplier = () -> 0.0;
    private DoubleSupplier omegaNormalizedSupplier = () -> 0.0;

    // Path following
    private Path currentPath = null;
    private boolean shouldResetPose = false;
    private Command currentPathCommand = null;
    private FollowPath.Builder followPathBuilder;

    // Omega override rotation
    private Rotation2d rotationRangeMin = Rotation2d.fromDegrees(-180);
    private Rotation2d rotationRangeMax = Rotation2d.fromDegrees(180);
    private Rotation2d snapTargetAngle = Rotation2d.fromDegrees(0);
    private PIDController omegaOverridePIDController;
    private PIDController snappedOmegaOverridePIDController;
    private static final double RANGED_ROTATION_MAX_VELOCITY_FACTOR = 0.6;
    private static final double RANGED_ROTATION_BUFFER_RAD = Math.toRadians(15.0); // Buffer to prevent oscillation at boundaries

    private boolean shouldOverrideOmega = false;
    private double omegaOverride = 0.0;
    private double lastUnoverriddenOmega = 0.0;

    // Translational speed freezing (used during shooting) - only vx/vy are frozen, omega remains controlled
    private boolean shouldOverrideTranslationalSpeedsFrozen = false;
    private double frozenVxMetersPerSec = 0.0;
    private double frozenVyMetersPerSec = 0.0;

    // Shooting velocity cap (limits max translational velocity during shooting)
    private boolean shouldOverrideVelocityCap = false;
    private double velocityCapMaxVelocityMetersPerSec = 1.0;

    // Alliance-based inversion
    private int invert = 1;

    private double modulesAlignmentToleranceDeg = 15.0;
    private Rotation2d modulesAlignmentTargetRotation = Rotation2d.fromDegrees(0);

    public static final double ODOMETRY_FREQUENCY = 250;

    public static final Lock odometryLock = new ReentrantLock();

    private ModuleIO[] modules;
    private ModuleIOInputsAutoLogged[] moduleInputs = {
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged()
    };

    private final GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };  
    private SwerveModuleState[] moduleStates = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private ChassisSpeeds desiredRobotRelativeSpeeds = new ChassisSpeeds();
    private ChassisSpeeds obtainableFieldRelativeSpeeds = new ChassisSpeeds();

    double prevLoopTime = Timer.getTimestamp();
    double prevDriveTime = Timer.getTimestamp();

    private final SwerveModuleGeneralConfig moduleGeneralConfig;
    private final SwerveDrivetrainConfig drivetrainConfig;
    private SwerveDriveKinematics kinematics;

    private final DashboardMotorControlLoopConfigurator driveControlLoopConfigurator;
    private final DashboardMotorControlLoopConfigurator steerControlLoopConfigurator;

    private final SysIdRoutine driveCharacterizationSysIdRoutine;
    private final SysIdRoutine steerCharacterizationSysIdRoutine;

    @SuppressWarnings("static-access")
    private SwerveDrive() {
        boolean useSimulation = Constants.shouldUseSimulation(Constants.SimOnlySubsystems.SWERVE);
        SwerveConfig swerveConfig = ConfigLoader.load(
            "swerve",
            ConfigLoader.getModeFolder(Constants.SimOnlySubsystems.SWERVE),
            SwerveConfig.class
        );
        drivetrainConfig = swerveConfig.drivetrain;
        moduleGeneralConfig = swerveConfig.moduleGeneral;

        if (useSimulation) {
            modules = new ModuleIO[] {
                new ModuleIOSim(moduleGeneralConfig, 0),
                new ModuleIOSim(moduleGeneralConfig, 1),
                new ModuleIOSim(moduleGeneralConfig, 2),
                new ModuleIOSim(moduleGeneralConfig, 3)
            };
            gyroIO = new GyroIO() {};
        } else if (Constants.currentMode == Constants.Mode.REPLAY) {
            modules = new ModuleIO[] {
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {}
            };
            gyroIO = new GyroIOPigeon2();
        } else {
            modules = new ModuleIO[] {
                new ModuleIOTalonFX(moduleGeneralConfig, swerveConfig.frontLeft),
                new ModuleIOTalonFX(moduleGeneralConfig, swerveConfig.frontRight),
                new ModuleIOTalonFX(moduleGeneralConfig, swerveConfig.backLeft),
                new ModuleIOTalonFX(moduleGeneralConfig, swerveConfig.backRight)
            };
            gyroIO = new GyroIOPigeon2();
            PhoenixOdometryThread.getInstance().start();
        }

        driveControlLoopConfigurator = new DashboardMotorControlLoopConfigurator(
            "Swerve/driveControlLoop",
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                moduleGeneralConfig.driveKP,
                moduleGeneralConfig.driveKI,
                moduleGeneralConfig.driveKD,
                moduleGeneralConfig.driveKS,
                moduleGeneralConfig.driveKV,
                moduleGeneralConfig.driveKA
            )
        );
        steerControlLoopConfigurator = new DashboardMotorControlLoopConfigurator(
            "Swerve/steerControlLoop",
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                moduleGeneralConfig.steerKP,
                moduleGeneralConfig.steerKI,
                moduleGeneralConfig.steerKD,
                moduleGeneralConfig.steerKS,
                moduleGeneralConfig.steerKV,
                moduleGeneralConfig.steerKA
            )
        );

        kinematics = new SwerveDriveKinematics(
            drivetrainConfig.getFrontLeftPositionMeters(),
            drivetrainConfig.getFrontRightPositionMeters(),
            drivetrainConfig.getBackLeftPositionMeters(),
            drivetrainConfig.getBackRightPositionMeters()
        );

        // Create the SysId routine - this is going to be in torque current foc units not voltage
        driveCharacterizationSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1.5).per(Second), Volts.of(12), Seconds.of(15), // Use default config
                (state) -> Logger.recordOutput("DriveCharacterizationSysIdRoutineState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (torqueCurrentFOC) -> {
                    for (ModuleIO module : modules) {
                        module.setDriveTorqueCurrentFOC(torqueCurrentFOC.in(Volts), new Rotation2d(0));
                    }
                },
                null, // No log consumer, since data is recorded by AdvantageKit
                this
            )
        );

        steerCharacterizationSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second), Volts.of(7), Seconds.of(10), // Use default config
                (state) -> Logger.recordOutput("SteerCharacterizationSysIdRoutineState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (torqueCurrentFOC) -> {
                    for (ModuleIO module : modules) {
                        module.setSteerTorqueCurrentFOC(torqueCurrentFOC.in(Volts), 0);
                    }
                },
                null, // No log consumer, since data is recorded by AdvantageKit
                this
            )
        );

        // Configure FollowPath builder using drivetrain config
        followPathBuilder = new FollowPath.Builder(
            this,
            RobotState.getInstance()::getEstimatedPose,
            RobotState.getInstance()::getRobotRelativeSpeeds,
            this::driveRobotRelative,
            new PIDController(
                drivetrainConfig.followPathTranslationKP,
                drivetrainConfig.followPathTranslationKI,
                drivetrainConfig.followPathTranslationKD
            ),
            new PIDController(
                drivetrainConfig.followPathRotationKP,
                drivetrainConfig.followPathRotationKI,
                drivetrainConfig.followPathRotationKD
            ),
            new PIDController(
                drivetrainConfig.followPathCrossTrackKP,
                drivetrainConfig.followPathCrossTrackKI,
                drivetrainConfig.followPathCrossTrackKD
            )
        ).withDefaultShouldFlip();

        // Configure omega override PID controllers with velocity limiting
        omegaOverridePIDController = new PIDController(
            drivetrainConfig.omegaOverrideKP,
            drivetrainConfig.omegaOverrideKI,
            drivetrainConfig.omegaOverrideKD
        );
        omegaOverridePIDController.enableContinuousInput(-Math.PI, Math.PI);
        omegaOverridePIDController.setTolerance(Math.toRadians(drivetrainConfig.rangedRotationToleranceDeg));

        snappedOmegaOverridePIDController = new PIDController(
            drivetrainConfig.omegaOverrideKP,
            drivetrainConfig.omegaOverrideKI,
            drivetrainConfig.omegaOverrideKD
        );
        snappedOmegaOverridePIDController.enableContinuousInput(-Math.PI, Math.PI);
        snappedOmegaOverridePIDController.setTolerance(Math.toRadians(drivetrainConfig.snappedToleranceDeg));

    }

    @Override
    public void periodic() {
        double dt = Timer.getTimestamp() - prevLoopTime; 
        prevLoopTime = Timer.getTimestamp();

        Logger.recordOutput("SwerveDrive/dtPeriodic", dt);

        // Thread safe reading of the gyro and swerve inputs.
        // The read lock is released only after inputs are written via the write lock
        odometryLock.lock();
        try {
            Logger.recordOutput("SwerveDrive/stateLockAcquired", true);
            gyroIO.updateInputs(gyroInputs);
            Logger.processInputs("SwerveDrive/gyro", gyroInputs);

            for (int i = 0; i < 4; i++) {
                modules[i].updateInputs(moduleInputs[i]);
                Logger.processInputs("SwerveDrive/module" + i, moduleInputs[i]);
            }
        } finally {
            odometryLock.unlock();
        }

        for (int i = 0; i < 4; i++) {
            moduleStates[i] = new SwerveModuleState(
                moduleInputs[i].driveVelocityMetersPerSec,
                moduleInputs[i].steerPosition
            );
        }

        if (driveControlLoopConfigurator.hasChanged()) {
            for (ModuleIO module : modules) {
                module.configureDriveControlLoop(driveControlLoopConfigurator.getConfig());
            }
        }
        if (steerControlLoopConfigurator.hasChanged()) {
            for (ModuleIO module : modules) {
                module.configureSteerControlLoop(steerControlLoopConfigurator.getConfig());
            }
        }

        ArrayList<Pose2d> updatedPoses = new ArrayList<Pose2d>();

        double[] odometryTimestampsSeconds = moduleInputs[0].odometryTimestampsSeconds;
        for (int i = 0; i < odometryTimestampsSeconds.length; i++) {
            for (int j = 0; j < 4; j++) {
                modulePositions[j] = new SwerveModulePosition(
                    moduleInputs[j].odometryDrivePositionsMeters[i],
                    moduleInputs[j].odometrySteerPositions[i]
                );
            }
            
            RobotState.getInstance().addOdometryObservation(
                new OdometryObservation(
                    odometryTimestampsSeconds[i],
                    gyroInputs.isConnected,
                    modulePositions,
                    moduleStates,
                    gyroInputs.isConnected ?
                        new Rotation3d(
                            gyroInputs.gyroOrientation.getX(),
                            gyroInputs.gyroOrientation.getY(),
                            gyroInputs.odometryYawPositions[i].getRadians()
                        ) :
                        new Rotation3d(),
                    gyroInputs.isConnected ? gyroInputs.yawVelocityRadPerSec : 0
                )
            );

            updatedPoses.add(RobotState.getInstance().getEstimatedPose());
        }

        Logger.recordOutput("SwerveDrive/updatedPoses", updatedPoses.toArray(new Pose2d[0]));
        Logger.recordOutput("SwerveDrive/measuredModuleStates", moduleStates);
        Logger.recordOutput("SwerveDrive/measuredModulePositions", modulePositions);

        // FSM processing
        handleStateTransitions();
        handleCurrentState();

        Logger.recordOutput("SwerveDrive/CurrentCommand", this.getCurrentCommand() == null ? "" : this.getCurrentCommand().toString());
    }

    /**
     * Determines the next measured state based on the desired state.
     */
    private void handleStateTransitions() {
        switch (desiredSystemState) {
            case DISABLED:
                currentSystemState = CurrentSystemState.DISABLED;
                break;
            case IDLE:
                currentSystemState = CurrentSystemState.IDLE;
                break;
            case TELEOP:
                currentSystemState = CurrentSystemState.TELEOP;
                break;

            case FOLLOW_PATH:
                if (currentPathCommand != null && currentPathCommand.isFinished()) {
                    currentSystemState = CurrentSystemState.IDLE;
                    desiredSystemState = DesiredSystemState.IDLE;
                } else {
                    currentSystemState = CurrentSystemState.FOLLOW_PATH;
                }
                break;
                
            case PREPARE_FOR_AUTO:
                if (currentPath != null) {
                    modulesAlignmentTargetRotation = currentPath.getInitialModuleDirection();
                    modulesAlignmentToleranceDeg = 15;
                }
                if (areModulesAligned()) {
                    currentSystemState = CurrentSystemState.READY_FOR_AUTO;
                } else {
                    currentSystemState = CurrentSystemState.PREPARE_FOR_AUTO;
                }
                break;

            case SYSID:
                currentSystemState = CurrentSystemState.SYSID;
                break;
        }

        switch (desiredOmegaOverrideState) {
            case NONE:
                currentOmegaOverrideState = CurrentOmegaOverrideState.NONE;
                break;
            case RANGED_ROTATION:
                if (isWithinRotationRange()) {
                    currentOmegaOverrideState = CurrentOmegaOverrideState.RANGED_NOMINAL;
                } else {
                    currentOmegaOverrideState = CurrentOmegaOverrideState.RANGED_RETURNING;
                }
                break;
            case SNAPPED:
                if (isAtSnapTarget()) {
                    currentOmegaOverrideState = CurrentOmegaOverrideState.SNAPPED_NOMINAL;
                } else {
                    currentOmegaOverrideState = CurrentOmegaOverrideState.SNAPPED_RETURNING;
                }
                break;
        }

        switch (desiredTranslationOverrideState) {
            case NONE:
                currentTranslationOverrideState = CurrentTranslationOverrideState.NONE;
                break;
            case FROZEN:
                currentTranslationOverrideState = CurrentTranslationOverrideState.FROZEN;
                break;
            case CAPPED:
                currentTranslationOverrideState = CurrentTranslationOverrideState.CAPPED;
                break;
        }
    }

    /**
     * Executes behavior for the current state.
     */
    private void handleCurrentState() {
        switch (currentSystemState) {
            case DISABLED:
                handleDISABLEDSystemState();
                break;
            case IDLE:
                handleIdleSystemState();
                break;
            case TELEOP:
                handleTeleopSystemState();
                break;
            case FOLLOW_PATH:
                handleFollowPathSystemState();
                break;
            case PREPARE_FOR_AUTO:
                handlePrepareForAutoSystemState();
                break;
            case READY_FOR_AUTO:
                handleReadyForAutoSystemState();
                break;
            case SYSID:
                handleSysIdSystemState();
                break;
        }

        switch (currentOmegaOverrideState) {
            case NONE:
                handleNoneOmegaOverrideState();
                break;
            case RANGED_NOMINAL:
                handleRangedRotationNominalOmegaOverrideState();
                break;
            case RANGED_RETURNING:
                handleRangedRotationReturningOmegaOverrideState();
                break;
            case SNAPPED_NOMINAL:
                handleSnappedNominalOmegaOverrideState();
                break;
            case SNAPPED_RETURNING:
                handleSnappedReturningOmegaOverrideState();
                break;
        }

        switch (currentTranslationOverrideState) {
            case NONE:
                handleNoneTranslationOverrideState();
                break;
            case FROZEN:
                handleFrozenTranslationOverrideState();
                break;
            case CAPPED:
                handleCappedTranslationOverrideState();
                break;
        }


    }
    private void cancelPathCommand() {
        if (currentPathCommand != null && currentPathCommand.isScheduled()) {
            currentPathCommand.cancel();
        }
        currentPathCommand = null;
    }

    private void handleDISABLEDSystemState() {
        if (previousSystemState != CurrentSystemState.DISABLED) {
            setWheelCoast(true);
        }
        cancelPathCommand();
        
        driveFieldRelative(new ChassisSpeeds(0, 0, 0));

        previousSystemState = CurrentSystemState.DISABLED;
    }

    private void handleIdleSystemState() {
        if (previousSystemState == CurrentSystemState.DISABLED) {
            setWheelCoast(false);
        }
        cancelPathCommand();

        // Only cancel running commands, but don't null out finished commands
        // This prevents re-scheduling when path completes while desired state is still FOLLOW_PATH
        if (currentPathCommand != null && currentPathCommand.isScheduled()) {
            currentPathCommand.cancel();
        }
        driveFieldRelative(new ChassisSpeeds(0, 0, 0));

        previousSystemState = CurrentSystemState.IDLE;
    }

    private void handleTeleopSystemState() {
        if (previousSystemState == CurrentSystemState.DISABLED) {
            setWheelCoast(false);
        }
        cancelPathCommand();

        invert = Constants.shouldFlipPath() ? -1 : 1;

        ChassisSpeeds desiredFieldRelativeSpeeds = new ChassisSpeeds(
            vxNormalizedSupplier.getAsDouble() * drivetrainConfig.maxTranslationalVelocityMetersPerSec * invert,
            vyNormalizedSupplier.getAsDouble() * drivetrainConfig.maxTranslationalVelocityMetersPerSec * invert,
            omegaNormalizedSupplier.getAsDouble() * drivetrainConfig.maxAngularVelocityRadiansPerSec
        );
        driveFieldRelative(desiredFieldRelativeSpeeds);

        previousSystemState = CurrentSystemState.TELEOP;
    }

    private void handleFollowPathSystemState() {
        if (previousSystemState == CurrentSystemState.DISABLED) {
            setWheelCoast(false);
        }
        
        // Schedule path command if not already running
        if (currentPath != null && (currentPathCommand == null || (!currentPathCommand.isScheduled() && !currentPathCommand.isFinished()))) {
            currentPathCommand = buildPathCommand(currentPath);
            CommandScheduler.getInstance().schedule(currentPathCommand);
        }
        // The path command handles driving via the callback

        previousSystemState = CurrentSystemState.FOLLOW_PATH;
    }

    private void prepareForAuto() {
        if (previousSystemState != CurrentSystemState.PREPARE_FOR_AUTO && previousSystemState != CurrentSystemState.READY_FOR_AUTO) {
            setWheelCoast(false);
        }
        cancelPathCommand();

        if (currentPath != null) {
            alignModules(currentPath.getInitialModuleDirection(), 15);
        }
    }

    private void handlePrepareForAutoSystemState() {
        prepareForAuto();
        previousSystemState = CurrentSystemState.PREPARE_FOR_AUTO;
    }

    private void handleReadyForAutoSystemState() {
        prepareForAuto();
        previousSystemState = CurrentSystemState.READY_FOR_AUTO;
    }

    private void handleSysIdSystemState() {
        if (previousSystemState == CurrentSystemState.DISABLED) {
            setWheelCoast(false);
        }
        cancelPathCommand();

        previousSystemState = CurrentSystemState.SYSID;
        // SysId routines handle their own motor control
        // This state just prevents other states from interfering
    }

    private void handleNoneOmegaOverrideState() {
        shouldOverrideOmega = false;
        omegaOverride = 0.0;

        previousOmegaOverrideState = CurrentOmegaOverrideState.NONE;
    }
    
    private void handleRangedRotationOmegaOverrideState() {
        shouldOverrideOmega = true;
        if (isWithinRotationRange(RANGED_ROTATION_BUFFER_RAD - Math.toRadians(drivetrainConfig.rangedRotationToleranceDeg))) {
            omegaOverride = limitOmegaForRange(lastUnoverriddenOmega); // TODO: BAD FIX
        } else {
            omegaOverride = calculateReturnToRangeOmega();
        }
    }

    private void handleRangedRotationNominalOmegaOverrideState() {
        handleRangedRotationOmegaOverrideState();
        previousOmegaOverrideState = CurrentOmegaOverrideState.RANGED_NOMINAL;
    }
    
    private void handleRangedRotationReturningOmegaOverrideState() {
        handleRangedRotationOmegaOverrideState();
        previousOmegaOverrideState = CurrentOmegaOverrideState.RANGED_RETURNING;
    }

    private void handleSnappedOmegaOverrideState() {
        shouldOverrideOmega = true;
        omegaOverride = calculateSnapOmega();
    }

    private void handleSnappedNominalOmegaOverrideState() {
        handleSnappedOmegaOverrideState();
        previousOmegaOverrideState = CurrentOmegaOverrideState.SNAPPED_NOMINAL;
    }

    private void handleSnappedReturningOmegaOverrideState() {
        handleSnappedOmegaOverrideState();
        previousOmegaOverrideState = CurrentOmegaOverrideState.SNAPPED_RETURNING;
    }

    private void handleNoneTranslationOverrideState() {
        shouldOverrideVelocityCap = false;
        shouldOverrideTranslationalSpeedsFrozen = false;

        previousTranslationOverrideState = CurrentTranslationOverrideState.NONE;
    }

    private void handleFrozenTranslationOverrideState() {
        shouldOverrideTranslationalSpeedsFrozen = true;
        shouldOverrideVelocityCap = false;

        if (previousTranslationOverrideState != CurrentTranslationOverrideState.FROZEN) {
            frozenVxMetersPerSec = RobotState.getInstance().getFieldRelativeSpeeds().vxMetersPerSecond;
            frozenVyMetersPerSec = RobotState.getInstance().getFieldRelativeSpeeds().vyMetersPerSecond;
        }

        previousTranslationOverrideState = CurrentTranslationOverrideState.FROZEN;
    }

    private void handleCappedTranslationOverrideState() {
        shouldOverrideVelocityCap = true;
        shouldOverrideTranslationalSpeedsFrozen = false;

        previousTranslationOverrideState = CurrentTranslationOverrideState.CAPPED;
    }

    /**
     * Builds a path command using the followPathBuilder, applying pose reset if configured.
     */
    private Command buildPathCommand(Path path) {
        if (shouldResetPose) {
            return followPathBuilder.withPoseReset(RobotState.getInstance()::resetPose).build(path);
        }
        return followPathBuilder.withPoseReset((Pose2d pose) -> {}).build(path);
    }

    /**
     * Checks if robot rotation is within the specified range.
     */
    private boolean isWithinRotationRange() {
        return isWithinRotationRange(0);
    }

    private boolean isAtSnapTarget() {
        Rotation2d currentRotation = RobotState.getInstance().getEstimatedPose().getRotation();
        double errorRad = MathUtil.angleModulus(snapTargetAngle.getRadians() - currentRotation.getRadians());
        return Math.abs(errorRad) <= Math.toRadians(drivetrainConfig.snappedToleranceDeg);
    }

    /**
     * Checks if robot rotation is within the specified range with an optional buffer.
     * @param buffer The buffer in radians to constrict the range by (applied to both min and max)
     */
    private boolean isWithinRotationRange(double buffer) {
        Rotation2d currentRotation = RobotState.getInstance().getEstimatedPose().getRotation();
        
        // Normalize angles to -PI to PI for comparison
        double current = MathUtil.angleModulus(currentRotation.getRadians());
        double min = MathUtil.angleModulus(rotationRangeMin.getRadians() + buffer);
        double max = MathUtil.angleModulus(rotationRangeMax.getRadians() - buffer);
        
        // Handle wrap-around case
        if (min <= max) {
            return current >= min && current <= max;
        } else {
            // Range wraps around (e.g., min=170deg, max=-170deg)
            return current >= min || current <= max;
        }
    }

    /**
     * Limits omega using sqrt(2*a*d) formula to prevent exceeding the padded rotation bounds.
     * Uses the padded range (user range constricted by buffer) as the boundary.
     * Accounts for current robot angular velocity to be more conservative when already
     * moving towards a boundary.
     */
    private double limitOmegaForRange(double desiredOmega) {
        Rotation2d currentRotation = RobotState.getInstance().getEstimatedPose().getRotation();
        double currentOmega = RobotState.getInstance().getYawVelocityRadPerSec();
        
        double current = currentRotation.getRadians();
        // Use padded range bounds (constricted by buffer)
        double min = rotationRangeMin.getRadians() + RANGED_ROTATION_BUFFER_RAD;
        double max = rotationRangeMax.getRadians() - RANGED_ROTATION_BUFFER_RAD;
        
        // Calculate distance to padded bounds (using Rotation2d for proper wrapping)
        double distToMin = Math.abs(MathUtil.angleModulus(current - min));
        double distToMax = Math.abs(MathUtil.angleModulus(max - current));
        
        double maxAngularAccel = drivetrainConfig.maxAngularAccelerationRadiansPerSecSec;
        
        // Calculate stopping distance from current velocity: d = ω² / (2α)
        double stoppingDistFromCurrent = (currentOmega * currentOmega) / (2 * maxAngularAccel);
        
        // Adjust effective distances based on current velocity direction
        // If moving towards a boundary, reduce effective distance by stopping distance
        double effectiveDistToMax = distToMax;
        double effectiveDistToMin = distToMin;
        
        if (currentOmega > 0) {
            // Moving towards max, reduce effective distance to max
            effectiveDistToMax = Math.max(0, distToMax - stoppingDistFromCurrent);
        } else if (currentOmega < 0) {
            // Moving towards min (negative omega), reduce effective distance to min
            effectiveDistToMin = Math.max(0, distToMin - stoppingDistFromCurrent);
        }
        
        // sqrt(2*a*d) formula for max velocity to stop at boundary using effective distances
        double maxOmegaToMin = Math.sqrt(2 * maxAngularAccel * effectiveDistToMin);
        double maxOmegaToMax = Math.sqrt(2 * maxAngularAccel * effectiveDistToMax);

        if (currentRotation.getRadians() < min) {
            return Math.max(desiredOmega, 0);
        }
        if (currentRotation.getRadians() > max) {
            return Math.min(desiredOmega, 0);
        }        

        // Clamp omega based on direction
        if (desiredOmega > 0) {
            // Rotating towards max bound
            return Math.min(desiredOmega, maxOmegaToMax);
        } else {
            // Rotating towards min bound
            return Math.max(desiredOmega, -maxOmegaToMin);
        }
    }

    /**
     * Calculates omega to return to rotation range using PID with velocity limiting.
     * Uses an internal buffer to target slightly inside the range to prevent oscillation at boundaries.
     */
    private double calculateReturnToRangeOmega() {
        Rotation2d currentRotation = RobotState.getInstance().getEstimatedPose().getRotation();
        
        double current = currentRotation.getRadians();
        double min = rotationRangeMin.getRadians();
        double max = rotationRangeMax.getRadians();
        
        // Find closest bound
        double distToMin = Math.abs(MathUtil.angleModulus(current - min));
        double distToMax = Math.abs(MathUtil.angleModulus(current - max));
        
        double targetAngle;
        if (distToMin < distToMax) {
            // Target slightly inside the min boundary (add buffer)
            targetAngle = min + RANGED_ROTATION_BUFFER_RAD;
        } else {
            // Target slightly inside the max boundary (subtract buffer)
            targetAngle = max - RANGED_ROTATION_BUFFER_RAD;
        }
        
        // Calculate PID output
        omegaOverridePIDController.setSetpoint(targetAngle);
        double pidOutput = omegaOverridePIDController.calculate(current);
        
        // Apply velocity limit (0.6 of max omega)
        double maxOmega = drivetrainConfig.maxAngularVelocityRadiansPerSec * RANGED_ROTATION_MAX_VELOCITY_FACTOR;

        
        return MathUtil.clamp(pidOutput, -maxOmega, maxOmega);
    }

    private double calculateSnapOmega() {
        Rotation2d currentRotation = RobotState.getInstance().getEstimatedPose().getRotation();
        double current = currentRotation.getRadians();
        double target = snapTargetAngle.getRadians();

        snappedOmegaOverridePIDController.setSetpoint(target);
        double pidOutput = snappedOmegaOverridePIDController.calculate(current);

        double maxOmega = drivetrainConfig.maxAngularVelocityRadiansPerSec;
        return MathUtil.clamp(pidOutput, -maxOmega, maxOmega);
    }

    // Supplier setters
    public void setTeleopInputSuppliers(
        DoubleSupplier vxNormalized,
        DoubleSupplier vyNormalized,
        DoubleSupplier omegaNormalized
    ) {
        this.vxNormalizedSupplier = vxNormalized;
        this.vyNormalizedSupplier = vyNormalized;
        this.omegaNormalizedSupplier = omegaNormalized;
    }

    public void setCurrentPath(Path path) {
        this.currentPath = path;
        this.shouldResetPose = false;
        // Clear existing command to allow fresh path to be scheduled
        cancelPathCommand();
    }

    public void setCurrentPath(Path path, boolean shouldResetPose) {
        this.currentPath = path;
        this.shouldResetPose = shouldResetPose;
        // Clear existing command to allow fresh path to be scheduled
        cancelPathCommand();
    }

    public void setRotationRange(Rotation2d min, Rotation2d max) {
        this.rotationRangeMin = min;
        this.rotationRangeMax = max;

        Logger.recordOutput("SwerveDrive/rotationRangeMin", min);
        Logger.recordOutput("SwerveDrive/rotationRangeMax", max);
    }

    public void setSnapTargetAngle(Rotation2d angle) {
        this.snapTargetAngle = angle;
        Logger.recordOutput("SwerveDrive/snapTargetAngle", angle);
    }

    public void setVelocityCapMaxVelocityMetersPerSec(double maxVelocity) {
        this.velocityCapMaxVelocityMetersPerSec = maxVelocity;
    }

    // System State getters/setters
    @AutoLogOutput(key = "SwerveDrive/desiredSystemState")
    public DesiredSystemState getDesiredSystemState() {
        return desiredSystemState;
    }

    @AutoLogOutput(key = "SwerveDrive/currentSystemState")
    public CurrentSystemState getCurrentSystemState() {
        return currentSystemState;
    }

    public void setDesiredSystemState(DesiredSystemState desiredState) {
        this.desiredSystemState = desiredState;
    }

    // Omega Override State getters/setters
    @AutoLogOutput(key = "SwerveDrive/desiredOmegaOverrideState")
    public DesiredOmegaOverrideState getDesiredOmegaOverrideState() {
        return desiredOmegaOverrideState;
    }

    @AutoLogOutput(key = "SwerveDrive/currentOmegaOverrideState")
    public CurrentOmegaOverrideState getCurrentOmegaOverrideState() {
        return currentOmegaOverrideState;
    }

    public void setDesiredOmegaOverrideState(DesiredOmegaOverrideState desiredOmegaOverrideState) {
        this.desiredOmegaOverrideState = desiredOmegaOverrideState;
    }

    // Translation Override State getters/setters
    @AutoLogOutput(key = "SwerveDrive/desiredTranslationOverrideState")
    public DesiredTranslationOverrideState getDesiredTranslationOverrideState() {
        return desiredTranslationOverrideState;
    }

    @AutoLogOutput(key = "SwerveDrive/currentTranslationOverrideState")
    public CurrentTranslationOverrideState getCurrentTranslationOverrideState() {
        return currentTranslationOverrideState;
    }

    public void setDesiredTranslationOverrideState(DesiredTranslationOverrideState desiredTranslationOverrideState) {
        this.desiredTranslationOverrideState = desiredTranslationOverrideState;
    }

    // Existing drive methods
    private ChassisSpeeds compensateRobotRelativeSpeeds(ChassisSpeeds speeds) {
        Rotation2d angularVelocity = new Rotation2d(speeds.omegaRadiansPerSecond * drivetrainConfig.rotationCompensationCoefficient);
        if (angularVelocity.getRadians() != 0.0) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                ChassisSpeeds.fromRobotRelativeSpeeds( // why should this be split into two?
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond,
                    RobotState.getInstance().getEstimatedPose().getRotation().plus(angularVelocity)
                ),
                RobotState.getInstance().getEstimatedPose().getRotation()
            );
        }

        return speeds;
    }

    // return a supplier that is true if the modules are aligned within the tolerance
    private void alignModules(Rotation2d targetRotation, double toleranceDeg) {
        modulesAlignmentTargetRotation = targetRotation;
        modulesAlignmentToleranceDeg = toleranceDeg;
        for (int i = 0; i < 4; i++) {
            SwerveModuleState state = new SwerveModuleState(0, targetRotation);
            state.optimize(moduleStates[i].angle);
            modules[i].setState(state);
        }
    }

    private boolean areModulesAligned() {
        for (int i = 0; i < 4; i++) {
            if (Math.abs(moduleStates[i].angle.minus(modulesAlignmentTargetRotation).getDegrees()) > modulesAlignmentToleranceDeg && 
                Math.abs(moduleStates[i].angle.plus(Rotation2d.fromDegrees(180)).minus(modulesAlignmentTargetRotation).getDegrees()) > modulesAlignmentToleranceDeg) {
                return false;
            }
        }
        return true;
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        double dt = Timer.getTimestamp() - prevDriveTime; 
        prevDriveTime = Timer.getTimestamp();

        lastUnoverriddenOmega = speeds.omegaRadiansPerSecond;
        desiredRobotRelativeSpeeds = speeds;

        if (shouldOverrideOmega) {
            desiredRobotRelativeSpeeds = new ChassisSpeeds(
                desiredRobotRelativeSpeeds.vxMetersPerSecond,
                desiredRobotRelativeSpeeds.vyMetersPerSecond,
                omegaOverride
            );
        }

        if (shouldOverrideVelocityCap) {
            double maxVelocity = velocityCapMaxVelocityMetersPerSec;
            double currentMagnitude = Math.hypot(desiredRobotRelativeSpeeds.vxMetersPerSecond, desiredRobotRelativeSpeeds.vyMetersPerSecond);
            if (currentMagnitude > maxVelocity && currentMagnitude > 0) {
                double scale = maxVelocity / currentMagnitude;
                desiredRobotRelativeSpeeds = new ChassisSpeeds(
                    desiredRobotRelativeSpeeds.vxMetersPerSecond * scale,
                    desiredRobotRelativeSpeeds.vyMetersPerSecond * scale,
                    desiredRobotRelativeSpeeds.omegaRadiansPerSecond
                );
            }
        }

        if (shouldOverrideTranslationalSpeedsFrozen) {
            ChassisSpeeds frozenFieldRelativeSpeeds = new ChassisSpeeds(
                frozenVxMetersPerSec,
                frozenVyMetersPerSec,
                desiredRobotRelativeSpeeds.omegaRadiansPerSecond
            );

            desiredRobotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(frozenFieldRelativeSpeeds, RobotState.getInstance().getEstimatedPose().getRotation());
        }

        ChassisSpeeds desiredFieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(desiredRobotRelativeSpeeds, RobotState.getInstance().getEstimatedPose().getRotation());
        Logger.recordOutput("SwerveDrive/desiredFieldRelativeSpeeds", desiredFieldRelativeSpeeds);
        Logger.recordOutput("SwerveDrive/desiredRobotRelativeSpeeds", desiredRobotRelativeSpeeds);
        
        // Limit acceleration to prevent sudden changes in speed
        obtainableFieldRelativeSpeeds = ChassisRateLimiter.limit(
            desiredFieldRelativeSpeeds, 
            obtainableFieldRelativeSpeeds, 
            dt, 
            drivetrainConfig.maxTranslationalAccelerationMetersPerSecSec, 
            drivetrainConfig.maxAngularAccelerationRadiansPerSecSec,
            drivetrainConfig.maxTranslationalVelocityMetersPerSec,
            drivetrainConfig.maxAngularVelocityRadiansPerSec
        );
        
        Logger.recordOutput("SwerveDrive/obtainableFieldRelativeSpeeds", obtainableFieldRelativeSpeeds);

        ChassisSpeeds obtainableRobotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(obtainableFieldRelativeSpeeds, RobotState.getInstance().getEstimatedPose().getRotation());
        Logger.recordOutput("SwerveDrive/obtainableRobotRelativeSpeeds", obtainableRobotRelativeSpeeds);

        SwerveModuleState[] moduleSetpoints = kinematics.toSwerveModuleStates(obtainableRobotRelativeSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleSetpoints, 
            drivetrainConfig.maxModuleVelocity
        );
        Logger.recordOutput("SwerveDrive/desaturatedModuleSetpoints", moduleSetpoints);

        for (int i = 0; i < 4; i++) {
            moduleSetpoints[i].optimize(moduleStates[i].angle);
            modules[i].setState(moduleSetpoints[i]);
        }
        Logger.recordOutput("SwerveDrive/optimizedModuleSetpoints", moduleSetpoints);
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, RobotState.getInstance().getEstimatedPose().getRotation());
        driveRobotRelative(speeds);
    }

    public void resetGyro(Rotation2d yaw) {
        gyroIO.resetGyro(yaw);
    }

    public Rotation2d getGyroAngle() {
        return gyroInputs.gyroOrientation.toRotation2d();
    }

    public void setWheelCoast(boolean isCoast) {
        for (ModuleIO module : modules) {
            module.setWheelCoast(isCoast);
        }
    }

    public Command getDynamicDriveCharacterizationSysIdRoutine(Direction direction) {
        return driveCharacterizationSysIdRoutine.dynamic(direction);
    }

    public Command getDynamicSteerCharacterizationSysIdRoutine(Direction direction) {
        return steerCharacterizationSysIdRoutine.dynamic(direction);
    }

    public Command getQuasistaticDriveCharacterizationSysIdRoutine(Direction direction) {
        return driveCharacterizationSysIdRoutine.quasistatic(direction);
    }

    public Command getQuasistaticSteerCharacterizationSysIdRoutine(Direction direction) {
        return steerCharacterizationSysIdRoutine.quasistatic(direction);
    }

    public FollowPath.Builder getFollowPathBuilder() {
        return followPathBuilder;
    }
}
