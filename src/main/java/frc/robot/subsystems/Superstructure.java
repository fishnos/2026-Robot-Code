package frc.robot.subsystems;

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

    public enum DesiredSystemState {
        DISABLED,
        HOME,
        TRACKING,
        READY_FOR_SHOT,
        SHOOTING,
        BUMP
    }

    public enum CurrentSystemState {
        DISABLED,
        HOME,
        TRACKING,
        PREPARING_FOR_SHOT,
        READY_FOR_SHOT,
        SHOOTING,
        BUMP
    }

    public enum DesiredIntakeState {
        DISABLED,
        STOWED,
        DEPLOYED,
        INTAKING,
        ALTERNATING
    }

    public enum CurrentIntakeState {
        DISABLED,
        STOWED,
        DEPLOYED,
        INTAKING,
        ALTERNATING
    }

    private DesiredSystemState desiredSystemState = DesiredSystemState.DISABLED;
    private CurrentSystemState currentSystemState = CurrentSystemState.DISABLED;
    private DesiredIntakeState desiredIntakeState = DesiredIntakeState.STOWED;
    private CurrentIntakeState currentIntakeState = CurrentIntakeState.DISABLED;

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
    
    // Cached shot data for mechanism control after applying guards.
    private ShotData cachedShotData;
    // Most recent raw shot calculation before guards.
    private ShotData mostRecentShotData;
    // Last shot calculation whose effective distance was inside configured min/max bounds.
    private ShotData lastInRangeShotData;
    private ShotReadinessData cachedShotReadinessData = new ShotReadinessData(
        new Translation3d(),
        new Translation3d(),
        Double.POSITIVE_INFINITY,
        0.0,
        0.0,
        0.0,
        false,
        false,
        false
    );

    private record ShotReadinessData(
        Translation3d actualLanding,
        Translation3d setpointLanding,
        double impactErrorMeters,
        double effectiveDistanceMeters,
        double minShotDistanceMeters,
        double maxShotDistanceMeters,
        boolean shooterReady,
        boolean distanceInRange,
        boolean readyForShot
    ) {}

    private Superstructure() {
        // Set up suppliers for the shooter - these provide dynamic setpoints based on shot calculation
        shooter.setHoodAngleSupplier(this::getTargetHoodAngle);
        shooter.setTurretAngleSupplier(this::getTargetTurretAngle);
        shooter.setFlywheelRPSSupplier(this::getTargetFlywheelRPS);
    }

    @Override
    public void periodic() {
        latencyCompensationSeconds.setDefault(shooter.getLatencyCompensationSeconds());

        // Calculate raw shot data once per cycle, then apply close-shot guard for mechanism setpoints.
        mostRecentShotData = calculateShotData();
        double minShotDistance = shooter.getMinShotDistFromShooterMeters();
        double maxShotDistance = shooter.getMaxShotDistFromShooterMeters();
        if (isDistanceInRange(mostRecentShotData.effectiveDistance(), minShotDistance, maxShotDistance)) {
            lastInRangeShotData = mostRecentShotData;
        }
        cachedShotData = selectShotDataWithMinDistanceGuard(
            mostRecentShotData,
            lastInRangeShotData,
            minShotDistance
        );
        Logger.recordOutput("Superstructure/shotData", cachedShotData);
        Logger.recordOutput("Superstructure/rawShotData", mostRecentShotData);
        Logger.recordOutput("Superstructure/usingCloseShotGuard", cachedShotData != mostRecentShotData);
        Logger.recordOutput("Superstructure/hasLastInRangeShotData", lastInRangeShotData != null);
        cachedShotReadinessData = calculateShotReadinessData();
        logShotReadinessData(cachedShotReadinessData);
        
        handleStateTransitions();
        handleCurrentState();
    }

    /**
     * Determines the next measured state based on the desired state.
     * Handles transitions between states with proper validation.
     */
    private void handleStateTransitions() {
        handleSystemStateTransitions();
        handleIntakeStateTransitions();
    }

    private void handleSystemStateTransitions() {
        if (currentSystemState == CurrentSystemState.SHOOTING
            && Timer.getTimestamp() - shotStartTime < SHOT_DURATION_SECONDS) {
            return; // If we're shooting, don't transition to another state until the shot is complete to prevent jitter
        }

        switch (desiredSystemState) {
            case DISABLED:
                currentSystemState = CurrentSystemState.DISABLED;
                break;

            case HOME:
                currentSystemState = CurrentSystemState.HOME;
                break;

            case TRACKING:
                currentSystemState = CurrentSystemState.TRACKING;
                break;

            case READY_FOR_SHOT:
                // READY_FOR_SHOT requires mechanisms to be at setpoints
                // Otherwise we're in PREPARING_FOR_SHOT
                if (isReadyForShot()) {
                    currentSystemState = CurrentSystemState.READY_FOR_SHOT;
                } else {
                    currentSystemState = CurrentSystemState.PREPARING_FOR_SHOT;
                }
                break;

            case SHOOTING:
                // SHOOTING only allowed when we're in READY_FOR_SHOT
                if (currentSystemState == CurrentSystemState.READY_FOR_SHOT) {
                    currentSystemState = CurrentSystemState.SHOOTING;
                    shotStartTime = Timer.getTimestamp();
                    hasStartedShooting = false;
                } else {
                    // Not ready yet, go to preparing
                    if (isReadyForShot()) {
                        if (currentSystemState == CurrentSystemState.SHOOTING) {
                            break;
                        }
                        currentSystemState = CurrentSystemState.SHOOTING;
                        shotStartTime = Timer.getTimestamp();
                        hasStartedShooting = false;
                    } else {
                        currentSystemState = CurrentSystemState.PREPARING_FOR_SHOT;
                    }
                }
                break;

            case BUMP:
                currentSystemState = CurrentSystemState.BUMP;
                break;
        }
    }

    private void handleIntakeStateTransitions() {
        if (currentSystemState == CurrentSystemState.DISABLED) {
            currentIntakeState = CurrentIntakeState.DISABLED;
            return;
        }
        if (currentSystemState == CurrentSystemState.HOME) {
            currentIntakeState = CurrentIntakeState.STOWED;
            return;
        }

        currentIntakeState = switch (desiredIntakeState) {
            case DISABLED -> CurrentIntakeState.DISABLED;
            case STOWED -> CurrentIntakeState.STOWED;
            case DEPLOYED -> CurrentIntakeState.DEPLOYED;
            case INTAKING -> CurrentIntakeState.INTAKING;
            case ALTERNATING -> CurrentIntakeState.ALTERNATING;
        };
    }

    /**
     * Executes behavior for the current state and sets subsystem states.
     */
    private void handleCurrentState() {
        switch (currentSystemState) {
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

        applyIntakeState(currentIntakeState);
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
                new VisualizeShot(cachedShotData.exitVelocity());
            } else {
                double ballIntervalSeconds = 1.0 / BALLS_PER_SECOND;
                if (now - lastBallVisualizedTime >= ballIntervalSeconds) {
                    lastBallVisualizedTime = now;
                    new VisualizeShot(cachedShotData.exitVelocity());
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

    private void applyIntakeState(CurrentIntakeState intakeState) {
        if (intakeState != CurrentIntakeState.ALTERNATING) {
            lastAlternatingIntakeToggleTime = 0;
        }

        switch (intakeState) {
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
     * Uses shot impact error and lerp table distance bounds as readiness metrics.
     */
    private boolean isReadyForShot() {
        return cachedShotReadinessData.readyForShot();
    }

    private ShotReadinessData calculateShotReadinessData() {
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
        double effectiveDistance = mostRecentShotData.effectiveDistance();
        double minShotDistance = shooter.getMinShotDistFromShooterMeters();
        double maxShotDistance = shooter.getMaxShotDistFromShooterMeters();
        boolean distanceInRange = isDistanceInRange(effectiveDistance, minShotDistance, maxShotDistance);
        boolean readyForShot = isShotReady(
            impactErrorMeters,
            SHOT_IMPACT_TOLERANCE_METERS,
            effectiveDistance,
            minShotDistance,
            maxShotDistance
        );

        return new ShotReadinessData(
            actualLanding,
            setpointLanding,
            impactErrorMeters,
            effectiveDistance,
            minShotDistance,
            maxShotDistance,
            shooterReady,
            distanceInRange,
            readyForShot
        );
    }

    private void logShotReadinessData(ShotReadinessData data) {
        Logger.recordOutput("Superstructure/shooterReady", data.shooterReady());
        Logger.recordOutput("Superstructure/shotDistanceInRange", data.distanceInRange());
        Logger.recordOutput("Superstructure/effectiveShotDistanceMeters", data.effectiveDistanceMeters());
        Logger.recordOutput("Superstructure/minShotDistanceMeters", data.minShotDistanceMeters());
        Logger.recordOutput("Superstructure/maxShotDistanceMeters", data.maxShotDistanceMeters());
        Logger.recordOutput("Superstructure/actualLanding", data.actualLanding());
        Logger.recordOutput("Superstructure/setpointLanding", data.setpointLanding());
        Logger.recordOutput("Superstructure/impactErrorMeters", data.impactErrorMeters());
    }

    static boolean isShotReady(
        double impactErrorMeters,
        double shotImpactToleranceMeters,
        double effectiveDistanceMeters,
        double minShotDistanceMeters,
        double maxShotDistanceMeters
    ) {
        boolean impactWithinTolerance = impactErrorMeters <= shotImpactToleranceMeters;
        boolean distanceInRange = isDistanceInRange(
            effectiveDistanceMeters,
            minShotDistanceMeters,
            maxShotDistanceMeters
        );
        return impactWithinTolerance && distanceInRange;
    }

    static boolean isDistanceInRange(
        double effectiveDistanceMeters,
        double minShotDistanceMeters,
        double maxShotDistanceMeters
    ) {
        return effectiveDistanceMeters >= minShotDistanceMeters
            && effectiveDistanceMeters <= maxShotDistanceMeters;
    }

    static ShotData selectShotDataWithMinDistanceGuard(
        ShotData mostRecentShotData,
        ShotData lastInRangeShotData,
        double minShotDistanceMeters
    ) {
        boolean shotIsTooClose = mostRecentShotData.effectiveDistance() < minShotDistanceMeters;
        if (shotIsTooClose && lastInRangeShotData != null) {
            return lastInRangeShotData;
        }
        return mostRecentShotData;
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
        Translation3d targetLocation = getFieldTargetLocation();
        
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
        
        // Get shooter offset from robot center (for omega compensation)
        Translation2d shooterOffsetFromRobotCenter = shooter.getShooterRelativePose().getTranslation().toTranslation2d();
        Rotation2d robotHeading = robotState.getEstimatedPose().getRotation();

        return ShotCalculator.calculate(
            targetLocation,
            shooterPosition,
            speeds,
            lerpTable,
            lcomp,
            shooter.getFlywheelVelocityRotationsPerSec(),
            shooter::calculateShotExitVelocityMetersPerSec,
            shooterOffsetFromRobotCenter,
            robotHeading
        );
    }

    private Translation3d getFieldTargetLocation() {
        Translation3d targetLocation = FieldConstants.Hub.hubCenter;
        if (Constants.shouldFlipPath()) {
            Translation2d fieldPosition = FlippingUtil.flipFieldPosition(targetLocation.toTranslation2d());
            targetLocation = new Translation3d(fieldPosition.getX(), fieldPosition.getY(), targetLocation.getZ());
        }
        return targetLocation;
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

        Translation3d targetLocation = getFieldTargetLocation();
        double shooterDistanceToTarget =
            shooterPosition.toTranslation2d().getDistance(targetLocation.toTranslation2d());
        InterpolatingMatrixTreeMap<Double, N3, N1> lerpTable = shooter.getLerpTable();
        double exitVelocity = ShotCalculator.calculateExitVelocityMetersPerSec(
            shooterDistanceToTarget,
            lerpTable,
            flywheelRPS,
            shooter::calculateShotExitVelocityMetersPerSec
        );
        double spinRateRadPerSec = shooter.calculateBackSpinRPM(flywheelRPS) * 2.0 * Math.PI / 60.0;

        double pitch = hoodAngleRotations * 2.0 * Math.PI;
        double turretYaw = turretAngleRotations * 2.0 * Math.PI;
        Rotation2d robotHeading = robotState.getEstimatedPose().getRotation();
        double fieldYaw = turretYaw + robotHeading.getRadians();

        double vHorizontal = exitVelocity * Math.cos(pitch);
        ChassisSpeeds fieldSpeeds = robotState.getFieldRelativeSpeeds();
        Translation2d shooterOffsetFromRobotCenter = shooter.getShooterRelativePose().getTranslation().toTranslation2d();
        double omega = fieldSpeeds.omegaRadiansPerSecond;
        double dx = shooterOffsetFromRobotCenter.getX();
        double dy = shooterOffsetFromRobotCenter.getY();
        double tangentialVxRobot = -omega * dy;
        double tangentialVyRobot = omega * dx;
        double tangentialVxField = tangentialVxRobot * robotHeading.getCos() - tangentialVyRobot * robotHeading.getSin();
        double tangentialVyField = tangentialVxRobot * robotHeading.getSin() + tangentialVyRobot * robotHeading.getCos();
        double shooterVxField = fieldSpeeds.vxMetersPerSecond + tangentialVxField;
        double shooterVyField = fieldSpeeds.vyMetersPerSecond + tangentialVyField;
        double vx = vHorizontal * Math.cos(fieldYaw) + shooterVxField;
        double vy = vHorizontal * Math.sin(fieldYaw) + shooterVyField;
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
    public void setDesiredSystemState(DesiredSystemState desiredSystemState) {
        this.desiredSystemState = desiredSystemState;
    }

    public void setDesiredIntakeState(DesiredIntakeState desiredIntakeState) {
        this.desiredIntakeState = desiredIntakeState;
    }

    @AutoLogOutput(key = "Superstructure/currentSystemState")
    public CurrentSystemState getCurrentSystemState() {
        return currentSystemState;
    }

    @AutoLogOutput(key = "Superstructure/desiredSystemState")
    public DesiredSystemState getDesiredSystemState() {
        return desiredSystemState;
    }

    @AutoLogOutput(key = "Superstructure/currentIntakeState")
    public CurrentIntakeState getCurrentIntakeState() {
        return currentIntakeState;
    }

    @AutoLogOutput(key = "Superstructure/desiredIntakeState")
    public DesiredIntakeState getDesiredIntakeState() {
        return desiredIntakeState;
    }
}
