package frc.robot.subsystems;

import java.util.function.DoubleUnaryOperator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.VisualizeShot;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.util.ballistics.BallisticsPhysics;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.util.ShotCalculator;
import frc.robot.lib.util.ShotCalculator.ShotData;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperSetpoint;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.FlywheelSetpoint;
import frc.robot.subsystems.shooter.Shooter.HoodSetpoint;
import frc.robot.subsystems.shooter.Shooter.TurretSetpoint;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeSetpoint;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance;
    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    private enum BaseState {
        DISABLED,
        HOME,
        TRACKING,
        PREPARING_FOR_SHOT,
        READY_FOR_SHOT,
        SHOOTING,
        BUMP
    }

    private enum IntakeMode {
        DISABLED,
        STOWED,
        DEPLOYED,
        INTAKING,
        ALTERNATING
    }

    public enum CurrentState {
        DISABLED(BaseState.DISABLED, IntakeMode.DISABLED),
        HOME(BaseState.HOME, IntakeMode.STOWED),
        TRACKING_STOWED(BaseState.TRACKING, IntakeMode.STOWED),
        TRACKING_DEPLOYED(BaseState.TRACKING, IntakeMode.DEPLOYED),
        TRACKING_INTAKING(BaseState.TRACKING, IntakeMode.INTAKING),
        PREPARING_FOR_SHOT_STOWED(BaseState.PREPARING_FOR_SHOT, IntakeMode.STOWED),
        PREPARING_FOR_SHOT_DEPLOYED(BaseState.PREPARING_FOR_SHOT, IntakeMode.DEPLOYED),
        PREPARING_FOR_SHOT_INTAKING(BaseState.PREPARING_FOR_SHOT, IntakeMode.INTAKING),
        PREPARING_FOR_SHOT_ALTERNATING(BaseState.PREPARING_FOR_SHOT, IntakeMode.ALTERNATING),
        READY_FOR_SHOT_STOWED(BaseState.READY_FOR_SHOT, IntakeMode.STOWED),
        READY_FOR_SHOT_DEPLOYED(BaseState.READY_FOR_SHOT, IntakeMode.DEPLOYED),
        READY_FOR_SHOT_INTAKING(BaseState.READY_FOR_SHOT, IntakeMode.INTAKING),
        READY_FOR_SHOT_ALTERNATING(BaseState.READY_FOR_SHOT, IntakeMode.ALTERNATING),
        SHOOTING_STOWED(BaseState.SHOOTING, IntakeMode.STOWED),
        SHOOTING_DEPLOYED(BaseState.SHOOTING, IntakeMode.DEPLOYED),
        SHOOTING_INTAKING(BaseState.SHOOTING, IntakeMode.INTAKING),
        SHOOTING_ALTERNATING(BaseState.SHOOTING, IntakeMode.ALTERNATING),
        BUMP_STOWED(BaseState.BUMP, IntakeMode.STOWED),
        BUMP_DEPLOYED(BaseState.BUMP, IntakeMode.DEPLOYED),
        BUMP_INTAKING(BaseState.BUMP, IntakeMode.INTAKING);

        public final BaseState baseState;
        public final IntakeMode intakeMode;

        CurrentState(BaseState baseState, IntakeMode intakeMode) {
            this.baseState = baseState;
            this.intakeMode = intakeMode;
        }
    }

    public enum DesiredState {
        DISABLED(BaseState.DISABLED, IntakeMode.DISABLED),
        HOME(BaseState.HOME, IntakeMode.STOWED),
        TRACKING_STOWED(BaseState.TRACKING, IntakeMode.STOWED),
        TRACKING_DEPLOYED(BaseState.TRACKING, IntakeMode.DEPLOYED),
        TRACKING_INTAKING(BaseState.TRACKING, IntakeMode.INTAKING),
        READY_FOR_SHOT_STOWED(BaseState.READY_FOR_SHOT, IntakeMode.STOWED),
        READY_FOR_SHOT_DEPLOYED(BaseState.READY_FOR_SHOT, IntakeMode.DEPLOYED),
        READY_FOR_SHOT_INTAKING(BaseState.READY_FOR_SHOT, IntakeMode.INTAKING),
        READY_FOR_SHOT_ALTERNATING(BaseState.READY_FOR_SHOT, IntakeMode.ALTERNATING),
        SHOOTING_STOWED(BaseState.SHOOTING, IntakeMode.STOWED),
        SHOOTING_DEPLOYED(BaseState.SHOOTING, IntakeMode.DEPLOYED),
        SHOOTING_INTAKING(BaseState.SHOOTING, IntakeMode.INTAKING),
        SHOOTING_ALTERNATING(BaseState.SHOOTING, IntakeMode.ALTERNATING),
        BUMP_STOWED(BaseState.BUMP, IntakeMode.STOWED),
        BUMP_DEPLOYED(BaseState.BUMP, IntakeMode.DEPLOYED),
        BUMP_INTAKING(BaseState.BUMP, IntakeMode.INTAKING);

        public final BaseState baseState;
        public final IntakeMode intakeMode;

        DesiredState(BaseState baseState, IntakeMode intakeMode) {
            this.baseState = baseState;
            this.intakeMode = intakeMode;
        }
    }

    private CurrentState currentState = CurrentState.DISABLED;
    private DesiredState desiredState = DesiredState.DISABLED;

    private final Shooter shooter = Shooter.getInstance();
    private final Hopper hopper = Hopper.getInstance();
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Intake intake = Intake.getInstance();
    private final RobotState robotState = RobotState.getInstance();

    private static final double SHOT_DURATION_SECONDS = .3; // Time to complete one shot
    private static final double ALTERNATING_INTAKE_TOGGLE_SECONDS = 1;
    private static final double BALLS_PER_SECOND = 12.0; // Balls per second to visualize

    private static final double SWERVE_ROTATION_MARGIN_DEG = 20.0; // Margin for swerve rotation range (degrees)
    private static final double MAX_TRANSLATIONAL_VELOCITY_DURING_SHOT_METERS_PER_SEC = 2.5;
    private static final double MAX_ANGULAR_VELOCITY_DURING_SHOT_RAD_PER_SEC = 0.8;
    private static final double SHOT_IMPACT_TOLERANCE_METERS = 0.3;
    private static final double BUMP_MAX_VELOCITY_METERS_PER_SEC = 1.8;
    private static final Rotation2d BUMP_SNAP_ANGLE = Rotation2d.fromDegrees(45);
    private LoggedNetworkNumber latencyCompensationSeconds = new LoggedNetworkNumber("Shooter/latencyCompSec");
    private double shotStartTime = 0;
    private double lastBallVisualizedTime = 0;
    private boolean hasStartedShooting = false;
    private double lastAlternatingIntakeToggleTime = 0;
    
    // Cached shot data - calculated once per periodic cycle
    private ShotData cachedShotData;

    private Superstructure() {
        // Set up suppliers for the shooter - these provide dynamic setpoints based on shot calculation
        shooter.setHoodAngleSupplier(this::getTargetHoodAngle);
        shooter.setTurretAngleSupplier(this::getTargetTurretAngle);
        shooter.setFlywheelRPSSupplier(this::getTargetFlywheelRPS);
    }

    @Override
    public void periodic() {
        // Calculate shot data once per cycle - all methods use this cached value
        cachedShotData = calculateShotData();
        Logger.recordOutput("Superstructure/shotData", cachedShotData);
        
        handleStateTransitions();
        handleCurrentState();
    }

    /**
     * Determines the next measured state based on the desired state.
     * Handles transitions between states with proper validation.
     */
    private void handleStateTransitions() {
        if (currentState.baseState == BaseState.SHOOTING && Timer.getTimestamp() - shotStartTime < SHOT_DURATION_SECONDS) {
            return; // If we're shooting, don't transition to another state until the shot is complete to prevent jitter
        }

        BaseState desiredBaseState = desiredState.baseState;
        IntakeMode desiredIntakeMode = desiredState.intakeMode;

        switch (desiredBaseState) {
            case DISABLED:
                currentState = resolveCurrentState(BaseState.DISABLED, desiredIntakeMode);
                break;

            case HOME:
                currentState = resolveCurrentState(BaseState.HOME, desiredIntakeMode);
                break;

            case TRACKING:
                currentState = resolveCurrentState(BaseState.TRACKING, desiredIntakeMode);
                break;

            case PREPARING_FOR_SHOT:
                currentState = resolveCurrentState(BaseState.PREPARING_FOR_SHOT, desiredIntakeMode);
                break;

            case READY_FOR_SHOT:
                // READY_FOR_SHOT requires mechanisms to be at setpoints
                // Otherwise we're in PREPARING_FOR_SHOT
                if (isReadyForShot()) {
                    currentState = resolveCurrentState(BaseState.READY_FOR_SHOT, desiredIntakeMode);
                } else {
                    currentState = resolveCurrentState(BaseState.PREPARING_FOR_SHOT, desiredIntakeMode);
                }
                break;

            case SHOOTING:
                // SHOOTING only allowed when we're in READY_FOR_SHOT
                if (currentState.baseState == BaseState.READY_FOR_SHOT) {
                    currentState = resolveCurrentState(BaseState.SHOOTING, desiredIntakeMode);
                    shotStartTime = Timer.getTimestamp();
                    hasStartedShooting = false;
                } else {
                    // Not ready yet, go to preparing
                    if (isReadyForShot()) {
                        if (currentState.baseState == BaseState.SHOOTING) {
                            break;
                        }
                        currentState = resolveCurrentState(BaseState.SHOOTING, desiredIntakeMode);
                        shotStartTime = Timer.getTimestamp();
                        hasStartedShooting = false;
                    } else {
                        currentState = resolveCurrentState(BaseState.PREPARING_FOR_SHOT, desiredIntakeMode);
                    }
                }
                break;

            case BUMP:
                currentState = resolveCurrentState(BaseState.BUMP, desiredIntakeMode);
                break;
        }

        latencyCompensationSeconds.setDefault(shooter.getLatencyCompensationSeconds());
    }

    /**
     * Executes behavior for the current state and sets subsystem states.
     */
    private void handleCurrentState() {
        switch (currentState.baseState) {
            case DISABLED:
                handleDisabledState();
                break;
            case HOME:
                handleHomeState();
                break;
            case TRACKING:
                handleTrackingState();
                break;
            case PREPARING_FOR_SHOT:
                handlePreparingForShotState();
                break;
            case READY_FOR_SHOT:
                handleReadyForShotState();
                break;
            case SHOOTING:
                handleShootingState();
                break;
            case BUMP:
                handleBumpState();
                break;
        }

        applyIntakeMode(currentState.intakeMode);
    }

    private void handleDisabledState() {
        shooter.setHoodSetpoint(HoodSetpoint.HOME);
        shooter.setTurretSetpoint(TurretSetpoint.HOME);
        shooter.setFlywheelSetpoint(FlywheelSetpoint.OFF);
        hopper.setSetpoint(HopperSetpoint.OFF);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.NONE);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.NONE);
    }

    private void handleHomeState() {
        shooter.setHoodSetpoint(HoodSetpoint.HOME);
        shooter.setTurretSetpoint(TurretSetpoint.HOME);
        shooter.setFlywheelSetpoint(FlywheelSetpoint.OFF);
        hopper.setSetpoint(HopperSetpoint.OFF);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.NONE);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.NONE);
    }

    private void handleTrackingState() {
        shooter.setHoodSetpoint(HoodSetpoint.DYNAMIC);
        shooter.setTurretSetpoint(TurretSetpoint.DYNAMIC);
        shooter.setFlywheelSetpoint(FlywheelSetpoint.OFF);
        hopper.setSetpoint(HopperSetpoint.OFF);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.NONE);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.NONE);
    }

    private void handlePreparingForShotState() {
        updateSwerveRotationRange();
        swerveDrive.setTranslationVelocityCapMaxVelocityMetersPerSec(MAX_TRANSLATIONAL_VELOCITY_DURING_SHOT_METERS_PER_SEC);
        swerveDrive.setOmegaVelocityCapMaxRadiansPerSec(MAX_ANGULAR_VELOCITY_DURING_SHOT_RAD_PER_SEC);

        shooter.setHoodSetpoint(HoodSetpoint.DYNAMIC);
        shooter.setTurretSetpoint(TurretSetpoint.DYNAMIC);
        shooter.setFlywheelSetpoint(FlywheelSetpoint.DYNAMIC);
        hopper.setSetpoint(HopperSetpoint.OFF);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.CAPPED);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.CAPPED);
    }

    private void handleReadyForShotState() {
        updateSwerveRotationRange();
        swerveDrive.setTranslationVelocityCapMaxVelocityMetersPerSec(MAX_TRANSLATIONAL_VELOCITY_DURING_SHOT_METERS_PER_SEC);
        swerveDrive.setOmegaVelocityCapMaxRadiansPerSec(MAX_ANGULAR_VELOCITY_DURING_SHOT_RAD_PER_SEC);

        shooter.setHoodSetpoint(HoodSetpoint.DYNAMIC);
        shooter.setTurretSetpoint(TurretSetpoint.DYNAMIC);
        shooter.setFlywheelSetpoint(FlywheelSetpoint.DYNAMIC);
        hopper.setSetpoint(HopperSetpoint.OFF);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.CAPPED);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.CAPPED);
    }

    private void handleShootingState() {        
        updateSwerveRotationRange();
        swerveDrive.setTranslationVelocityCapMaxVelocityMetersPerSec(MAX_TRANSLATIONAL_VELOCITY_DURING_SHOT_METERS_PER_SEC);
        swerveDrive.setOmegaVelocityCapMaxRadiansPerSec(MAX_ANGULAR_VELOCITY_DURING_SHOT_RAD_PER_SEC);

        shooter.setHoodSetpoint(HoodSetpoint.DYNAMIC);
        shooter.setTurretSetpoint(TurretSetpoint.DYNAMIC);
        shooter.setFlywheelSetpoint(FlywheelSetpoint.DYNAMIC);
        hopper.setSetpoint(HopperSetpoint.FEEDING);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.CAPPED);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.CAPPED);

        double now = Timer.getTimestamp();
        double timeSinceShotStart = now - shotStartTime;
        if (timeSinceShotStart >= SHOT_DURATION_SECONDS) {
            if (!hasStartedShooting) {
                hasStartedShooting = true;
                lastBallVisualizedTime = now;
                new VisualizeShot();
            } else {
                double ballIntervalSeconds = 1.0 / BALLS_PER_SECOND;
                if (now - lastBallVisualizedTime >= ballIntervalSeconds) {
                    lastBallVisualizedTime = now;
                    new VisualizeShot();
                }
            }
        }
    }

    private void handleBumpState() {
        swerveDrive.setSnapTargetAngle(BUMP_SNAP_ANGLE);
        swerveDrive.setTranslationVelocityCapMaxVelocityMetersPerSec(BUMP_MAX_VELOCITY_METERS_PER_SEC);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.SNAPPED);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.CAPPED);
        
        // Keep shooter in home position during bump
        shooter.setHoodSetpoint(HoodSetpoint.HOME);
        shooter.setTurretSetpoint(TurretSetpoint.HOME);
        shooter.setFlywheelSetpoint(FlywheelSetpoint.OFF);
        hopper.setSetpoint(HopperSetpoint.OFF);
    }

    private CurrentState resolveCurrentState(BaseState baseState, IntakeMode intakeMode) {
        return switch (baseState) {
            case DISABLED -> CurrentState.DISABLED;
            case HOME -> CurrentState.HOME;
            case TRACKING -> switch (intakeMode) {
                case STOWED -> CurrentState.TRACKING_STOWED;
                case DEPLOYED -> CurrentState.TRACKING_DEPLOYED;
                case INTAKING -> CurrentState.TRACKING_INTAKING;
                case DISABLED, ALTERNATING -> CurrentState.TRACKING_STOWED;
            };
            case PREPARING_FOR_SHOT -> switch (intakeMode) {
                case STOWED -> CurrentState.PREPARING_FOR_SHOT_STOWED;
                case DEPLOYED -> CurrentState.PREPARING_FOR_SHOT_DEPLOYED;
                case INTAKING -> CurrentState.PREPARING_FOR_SHOT_INTAKING;
                case ALTERNATING -> CurrentState.PREPARING_FOR_SHOT_ALTERNATING;
                case DISABLED -> CurrentState.PREPARING_FOR_SHOT_STOWED;
            };
            case READY_FOR_SHOT -> switch (intakeMode) {
                case STOWED -> CurrentState.READY_FOR_SHOT_STOWED;
                case DEPLOYED -> CurrentState.READY_FOR_SHOT_DEPLOYED;
                case INTAKING -> CurrentState.READY_FOR_SHOT_INTAKING;
                case ALTERNATING -> CurrentState.READY_FOR_SHOT_ALTERNATING;
                case DISABLED -> CurrentState.READY_FOR_SHOT_STOWED;
            };
            case SHOOTING -> switch (intakeMode) {
                case STOWED -> CurrentState.SHOOTING_STOWED;
                case DEPLOYED -> CurrentState.SHOOTING_DEPLOYED;
                case INTAKING -> CurrentState.SHOOTING_INTAKING;
                case ALTERNATING -> CurrentState.SHOOTING_ALTERNATING;
                case DISABLED -> CurrentState.SHOOTING_STOWED;
            };
            case BUMP -> switch (intakeMode) {
                case STOWED -> CurrentState.BUMP_STOWED;
                case DEPLOYED -> CurrentState.BUMP_DEPLOYED;
                case INTAKING -> CurrentState.BUMP_INTAKING;
                case DISABLED, ALTERNATING -> CurrentState.BUMP_STOWED;
            };
        };
    }

    private void applyIntakeMode(IntakeMode intakeMode) {
        if (intakeMode != IntakeMode.ALTERNATING) {
            lastAlternatingIntakeToggleTime = 0;
        }

        switch (intakeMode) {
            case DISABLED:
                intake.setSetpoint(IntakeSetpoint.DISABLED);
                break;
            case STOWED:
                intake.setSetpoint(IntakeSetpoint.STOWED);
                break;
            case DEPLOYED:
                intake.setSetpoint(IntakeSetpoint.DEPLOYED);
                break;
            case INTAKING:
                intake.setSetpoint(IntakeSetpoint.INTAKING);
                break;
            case ALTERNATING:
                applyAlternatingIntake();
                break;
        }
    }

    private void applyAlternatingIntake() {
        double now = Timer.getTimestamp();
        if (lastAlternatingIntakeToggleTime == 0) {
            lastAlternatingIntakeToggleTime = now;
        }

        boolean timerElapsed = now - lastAlternatingIntakeToggleTime >= ALTERNATING_INTAKE_TOGGLE_SECONDS;
        boolean pivotSettled = intake.isPivotAtSetpoint();
        if (timerElapsed && pivotSettled) {
            lastAlternatingIntakeToggleTime = now;
            intake.setSetpoint(intake.isStowed() ? IntakeSetpoint.DEPLOYED : IntakeSetpoint.STOWED);
            return;
        }

        intake.setSetpoint(intake.isStowed() ? IntakeSetpoint.STOWED : IntakeSetpoint.DEPLOYED);
    }

    /**
     * Checks if all mechanisms are at their setpoints and ready to shoot.
     * Uses shot impact error as the sole readiness metric.
     */
    private boolean isReadyForShot() {
        double actualHood = shooter.getHoodAngleRotations();
        double actualTurret = shooter.getTurretAngleRotations();
        double actualFlywheel = shooter.getFlywheelVelocityRotationsPerSec();

        double setpointHood = getTargetHoodAngle().getRotations();
        double setpointTurret = getTargetTurretAngle().getRotations();
        double setpointFlywheel = getTargetFlywheelRPS();

        Translation3d actualLanding = simulateShotLanding(actualHood, actualTurret, actualFlywheel);
        Translation3d setpointLanding = simulateShotLanding(setpointHood, setpointTurret, setpointFlywheel);
        double impactErrorMeters = actualLanding.toTranslation2d()
            .getDistance(setpointLanding.toTranslation2d());
        boolean shooterReady = impactErrorMeters <= SHOT_IMPACT_TOLERANCE_METERS;
        Logger.recordOutput("Superstructure/shooterReady", shooterReady);
        Logger.recordOutput("Superstructure/actualLanding", actualLanding);
        Logger.recordOutput("Superstructure/setpointLanding", setpointLanding);
        Logger.recordOutput("Superstructure/impactErrorMeters", impactErrorMeters);

        return shooterReady;
    }

    /**
     * Updates the swerve rotation range based on turret limits and target shot angle.
     * This ensures the robot heading keeps the turret within its physical limits.
     */
    private void updateSwerveRotationRange() {
        double targetFieldYawDeg = cachedShotData.targetFieldYaw().getDegrees();
        
        // Get turret physical limits
        double turretMinDeg = shooter.getTurretMinAngleDeg();
        double turretMaxDeg = shooter.getTurretMaxAngleDeg();
        
        // Calculate robot rotation range
        // Robot rotation = target field yaw - turret angle
        // So: turret angle = target field yaw - robot rotation
        // For turret to be within [turretMin, turretMax]:
        // Robot rotation must be within [targetYaw - turretMax, targetYaw - turretMin]
        // Add margin to ensure we stay well within bounds
        double robotRotationMinOffsetDeg = targetFieldYawDeg - (turretMaxDeg - SWERVE_ROTATION_MARGIN_DEG);
        double robotRotationMaxOffsetDeg = targetFieldYawDeg - (turretMinDeg + SWERVE_ROTATION_MARGIN_DEG);
        
        swerveDrive.setRotationRangeOffsetDegrees(robotRotationMinOffsetDeg, robotRotationMaxOffsetDeg);
    }

    // Target suppliers for shooter - these use cached shot data
    // Target suppliers always provide values from shot calculator
    // Shooter decides when to use them based on its state
    private Rotation2d getTargetHoodAngle() {
        return cachedShotData.hoodPitch();
    }

    private Rotation2d getTargetTurretAngle() {
        return cachedShotData.targetFieldYaw().minus(robotState.getEstimatedPose().getRotation());
    }

    private double getTargetFlywheelRPS() {
        return cachedShotData.flywheelRPS();
    }

    private ShotData calculateShotData() {
        Translation3d targetLocation = FieldConstants.Hub.hubCenter;
        if (Constants.shouldFlipPath()) {
            Translation2d fieldPosition = targetLocation.toTranslation2d();
            fieldPosition = FlippingUtil.flipFieldPosition(fieldPosition);

            targetLocation = new Translation3d(fieldPosition.getX(), fieldPosition.getY(), targetLocation.getZ());
        }
        
        // Calculate shooter position in field coordinates
        Pose3d robotPose = new Pose3d(
            new Translation3d(
                robotState.getEstimatedPose().getX(), 
                robotState.getEstimatedPose().getY(), 
                0
            ),
            new Rotation3d(0, 0, robotState.getEstimatedPose().getRotation().getRadians())
        );
        Translation3d shooterPosition = robotPose.plus(new Transform3d(new Pose3d(), shooter.getShooterRelativePose())).getTranslation();
        
        ChassisSpeeds speeds = robotState.getFieldRelativeSpeeds();
        InterpolatingMatrixTreeMap<Double, N3, N1> lerpTable = shooter.getLerpTable();
        double lcomp = latencyCompensationSeconds.get();
        DoubleUnaryOperator rpsToExitVelocity = shooter::calculateShotExitVelocityMetersPerSec;
        DoubleUnaryOperator rpsToSpinRateRadPerSec =
            rps -> shooter.calculateBackSpinRPM(rps) * 2.0 * Math.PI / 60.0;
        
        // Get shooter offset from robot center (for omega compensation)
        Translation2d shooterOffsetFromRobotCenter = shooter.getShooterRelativePose().getTranslation().toTranslation2d();
        Rotation2d robotHeading = robotState.getEstimatedPose().getRotation();

        return ShotCalculator.calculate(
            targetLocation,
            shooterPosition,
            speeds,
            lerpTable,
            lcomp,
            rpsToExitVelocity,
            rpsToSpinRateRadPerSec,
            shooterOffsetFromRobotCenter,
            robotHeading
        );
    }

    private Translation3d simulateShotLanding(
        double hoodAngleRotations,
        double turretAngleRotations,
        double flywheelRPS
    ) {
        Pose3d robotPose = new Pose3d(
            new Translation3d(
                robotState.getEstimatedPose().getX(),
                robotState.getEstimatedPose().getY(),
                0
            ),
            new Rotation3d(0, 0, robotState.getEstimatedPose().getRotation().getRadians())
        );
        Translation3d shooterPosition =
            robotPose.plus(new Transform3d(new Pose3d(), shooter.getShooterRelativePose())).getTranslation();

        double exitVelocity = shooter.calculateShotExitVelocityMetersPerSec(flywheelRPS);
        double spinRateRadPerSec = shooter.calculateBackSpinRPM(flywheelRPS) * 2.0 * Math.PI / 60.0;

        double pitch = hoodAngleRotations * 2.0 * Math.PI;
        double turretYaw = turretAngleRotations * 2.0 * Math.PI;
        double fieldYaw = turretYaw + robotState.getEstimatedPose().getRotation().getRadians();

        double vHorizontal = exitVelocity * Math.cos(pitch);
        ChassisSpeeds fieldSpeeds = robotState.getFieldRelativeSpeeds();
        double vx = vHorizontal * Math.cos(fieldYaw) + fieldSpeeds.vxMetersPerSecond;
        double vy = vHorizontal * Math.sin(fieldYaw) + fieldSpeeds.vyMetersPerSecond;
        double vz = exitVelocity * Math.sin(pitch);

        double hubZ = FieldConstants.Hub.hubCenter.getZ();
        return BallisticsPhysics.simulateToHeight3D(
            shooterPosition,
            vx,
            vy,
            vz,
            spinRateRadPerSec,
            hubZ,
            0.002
        );
    }

    // Public interface
    public void setDesiredState(DesiredState desiredState) {
        this.desiredState = desiredState;
    }

    @AutoLogOutput(key = "Superstructure/currentState")
    public CurrentState getCurrentState() {
        return currentState;
    }

    @AutoLogOutput(key = "Superstructure/desiredState")
    public DesiredState getDesiredState() {
        return desiredState;
    }
}
