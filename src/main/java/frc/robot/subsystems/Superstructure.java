package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.VisualizeShot;
import frc.robot.configs.SuperstructureConfig;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ZoneConstants;
import frc.robot.constants.ZoneConstants.RectangleZone;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.util.ConfigLoader;
import frc.robot.lib.util.ShotKinematicsUtil;
import frc.robot.lib.util.ShotCalculator;
import frc.robot.lib.util.ShotCalculator.ShotData;
import frc.robot.lib.util.ZoneUtil;
import frc.robot.subsystems.climber.Climber;
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
    private static final double TARGET_HEIGHT_REACH_EPSILON_METERS = 5e-4;

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
        REVERSING,
        ALTERNATING
    }

    public enum CurrentIntakeState {
        DISABLED,
        STOWED,
        DEPLOYED,
        INTAKING,
        REVERSING,
        ALTERNATING
    }

    public enum DesiredClimbState {
        DISABLED,
        RETRACTED,
        EXTENDED,
        CLIMBED
    }

    public enum CurrentClimbState {
        DISABLED,
        RETRACTED,
        EXTENDED,
        CLIMBED
    }

    public enum TargetState {
        HUB,
        PASS_ALLIANCE_TOP,
        PASS_ALLIANCE_BOTTOM,
        PASS_NEUTRAL_TOP,
        PASS_NEUTRAL_BOTTOM;

        public boolean isPassTarget() {
            return this != HUB;
        }

        public boolean isAlliancePassTarget() {
            return switch (this) {
                case PASS_ALLIANCE_TOP, PASS_ALLIANCE_BOTTOM -> true;
                default -> false;
            };
        }
    }

    enum RobotFieldZone {
        CURRENT_ALLIANCE,
        NEUTRAL,
        OPPOSING_ALLIANCE,
        UNKNOWN
    }

    private DesiredSystemState desiredSystemState = DesiredSystemState.DISABLED;
    private CurrentSystemState currentSystemState = CurrentSystemState.DISABLED;
    private CurrentSystemState lastStateBeforePreparingForShot = CurrentSystemState.DISABLED;
    private DesiredIntakeState desiredIntakeState = DesiredIntakeState.STOWED;
    private CurrentIntakeState currentIntakeState = CurrentIntakeState.DISABLED;
    private DesiredClimbState desiredClimbState = DesiredClimbState.RETRACTED;
    private CurrentClimbState currentClimbState = CurrentClimbState.DISABLED;
    private TargetState desiredTargetState = TargetState.HUB;
    private TargetState currentTargetState = TargetState.HUB;

    private final Shooter shooter = Shooter.getInstance();
    private final Hopper hopper = Hopper.getInstance();
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Climber climber = Climber.getInstance();
    private final RobotState robotState = RobotState.getInstance();
    private final SuperstructureConfig config;
    private final Rotation2d bumpSnapAngle;

    private final LoggedNetworkNumber latencyCompensationSeconds = new LoggedNetworkNumber("Shooter/latencyCompSec");
    private double shotStartTime = 0;
    private double lastBallVisualizedTime = 0;
    private boolean hasStartedShooting = false;
    private double lastAlternatingIntakeToggleTime = 0;
    private boolean hasWarnedInvalidTurretRotationMargins = false;
    
    // Cached shot data for mechanism control after applying guards.
    private ShotData cachedShotData;
    // Most recent raw shot calculation before guards.
    private ShotData mostRecentShotData;
    // Last shot calculation whose effective distance was inside configured min/max bounds.
    private ShotData lastInRangeShotData;
    private TargetState lastInRangeShotTargetState = TargetState.HUB;
    private double lastInRangeShotTimestampSeconds = Double.NEGATIVE_INFINITY;
    private ShotComputationContext cachedShotComputationContext;
    private ShotReadinessData cachedShotReadinessData = new ShotReadinessData(
        0.0,
        0.0,
        0.0,
        false,
        false,
        false,
        false,
        false,
        0.0,
        0.0,
        0.0,
        0.0,
        false,
        false,
        true,
        true,
        false
    );

    private record ShotReadinessData(
        double effectiveDistanceMeters,
        double minShotDistanceMeters,
        double maxShotDistanceMeters,
        boolean hoodAtSetpoint,
        boolean turretAtSetpoint,
        boolean turretFieldRelativeAtSetpoint,
        boolean flywheelAtSetpoint,
        boolean swerveOmegaCapped,
        double hoodErrorDegrees,
        double turretErrorDegrees,
        double turretFieldRelativeErrorDegrees,
        double flywheelErrorRps,
        boolean shooterReady,
        boolean distanceInRange,
        boolean lineOfSightClear,
        boolean zoneAllowsTarget,
        boolean readyForShot
    ) {}

    private record ShotComputationContext(
        TargetState targetState,
        Translation3d targetLocation,
        ShotKinematicsUtil.ShooterKinematics shooterKinematics,
        InterpolatingMatrixTreeMap<Double, N3, N1> lerpTable,
        double minDistanceMeters,
        double maxDistanceMeters,
        boolean passingTarget,
        boolean lineOfSightClear,
        boolean zoneAllowsTarget
    ) {}

    record TurretRotationMargins(
        double marginTowardMinDeg,
        double marginTowardMaxDeg,
        boolean valid
    ) {}

    record AccumulatedYawRotationRange(
        double minAbsDeg,
        double maxAbsDeg
    ) {}

    record TargetCandidate(
        TargetState targetState,
        Translation2d targetPosition,
        boolean lineOfSightClear
    ) {}

    private Superstructure() {
        config = ConfigLoader.load("superstructure", SuperstructureConfig.class);
        bumpSnapAngle = Rotation2d.fromDegrees(config.bumpSnapAngleDegrees);

        // Set up suppliers for the shooter - these provide dynamic setpoints based on shot calculation
        shooter.setHoodAngleSupplier(this::getTargetHoodAngle);
        shooter.setTurretAngleSupplier(this::getTargetTurretAngle);
        shooter.setTurretVelocitySupplier(this::getTargetTurretVelocityRotPerSec);
        shooter.setFlywheelRPSSupplier(this::getTargetFlywheelRPS);
    }

    @Override
    public void periodic() {
        latencyCompensationSeconds.setDefault(config.latencyCompensationSeconds);

        // Calculate raw shot data once per cycle, then apply close-shot guard for mechanism setpoints.
        cachedShotComputationContext = buildShotComputationContext();
        mostRecentShotData = calculateShotData(cachedShotComputationContext);
        double nowSeconds = Timer.getTimestamp();
        if (cachedShotComputationContext.targetState() != lastInRangeShotTargetState) {
            lastInRangeShotData = null;
            lastInRangeShotTimestampSeconds = Double.NEGATIVE_INFINITY;
            lastInRangeShotTargetState = cachedShotComputationContext.targetState();
        }
        double minShotDistance = cachedShotComputationContext.minDistanceMeters();
        double maxShotDistance = cachedShotComputationContext.maxDistanceMeters();
        if (isDistanceInRange(mostRecentShotData.effectiveDistance(), minShotDistance, maxShotDistance)) {
            lastInRangeShotData = mostRecentShotData;
            lastInRangeShotTimestampSeconds = nowSeconds;
            lastInRangeShotTargetState = cachedShotComputationContext.targetState();
        }
        boolean hasValidLastInRangeShotData = isLastInRangeShotDataValid(
            nowSeconds,
            lastInRangeShotTimestampSeconds,
            config.lastInRangeShotMaxAgeSeconds
        );
        if (!hasValidLastInRangeShotData) {
            lastInRangeShotData = null;
        }
        cachedShotData = selectShotDataWithMinDistanceGuard(
            mostRecentShotData,
            hasValidLastInRangeShotData ? lastInRangeShotData : null,
            minShotDistance
        );
        Logger.recordOutput("Superstructure/shotData", cachedShotData);
        Logger.recordOutput("Superstructure/rawShotData", mostRecentShotData);
        Logger.recordOutput("Superstructure/usingCloseShotGuard", cachedShotData != mostRecentShotData);
        Logger.recordOutput("Superstructure/hasLastInRangeShotData", hasValidLastInRangeShotData);
        Logger.recordOutput("Superstructure/lastInRangeShotAgeSeconds", nowSeconds - lastInRangeShotTimestampSeconds);

        cachedShotReadinessData = calculateShotReadinessData(cachedShotComputationContext);
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
        handleClimbStateTransitions();
    }

    private void handleSystemStateTransitions() {
        if (currentSystemState == CurrentSystemState.SHOOTING
            && Timer.getTimestamp() - shotStartTime < config.shotDurationSeconds) {
            return; // If we're shooting, don't transition to another state until the shot is complete to prevent jitter
        }

        CurrentSystemState nextSystemState = currentSystemState;

        switch (desiredSystemState) {
            case DISABLED:
                nextSystemState = CurrentSystemState.DISABLED;
                break;

            case HOME:
                nextSystemState = CurrentSystemState.HOME;
                break;

            case TRACKING:
                nextSystemState = CurrentSystemState.TRACKING;
                break;

            case READY_FOR_SHOT:
                // READY_FOR_SHOT requires mechanisms to be at setpoints
                // Otherwise we're in PREPARING_FOR_SHOT
                if (isReadyForShot()) {
                    nextSystemState = CurrentSystemState.READY_FOR_SHOT;
                } else {
                    nextSystemState = CurrentSystemState.PREPARING_FOR_SHOT;
                }
                break;

            case SHOOTING:
                boolean shouldStartShooting = currentSystemState == CurrentSystemState.READY_FOR_SHOT
                    || (currentSystemState != CurrentSystemState.SHOOTING && isReadyForShot());
                if (shouldStartShooting) {
                    nextSystemState = CurrentSystemState.SHOOTING;
                    shotStartTime = Timer.getTimestamp();
                    hasStartedShooting = false;
                } else if (!isReadyForShot()) {
                    nextSystemState = CurrentSystemState.PREPARING_FOR_SHOT;
                }
                break;

            case BUMP:
                nextSystemState = CurrentSystemState.BUMP;
                break;
        }

        if (nextSystemState == CurrentSystemState.PREPARING_FOR_SHOT
            && currentSystemState != CurrentSystemState.PREPARING_FOR_SHOT) {
            lastStateBeforePreparingForShot = currentSystemState;
        }

        currentSystemState = nextSystemState;
    }

    private void handleIntakeStateTransitions() {
        if (currentSystemState == CurrentSystemState.DISABLED) {
            currentIntakeState = CurrentIntakeState.DISABLED;
            return;
        }

        currentIntakeState = switch (desiredIntakeState) {
            case DISABLED -> CurrentIntakeState.DISABLED;
            case STOWED -> CurrentIntakeState.STOWED;
            case DEPLOYED -> CurrentIntakeState.DEPLOYED;
            case INTAKING -> CurrentIntakeState.INTAKING;
            case REVERSING -> CurrentIntakeState.REVERSING;
            case ALTERNATING -> CurrentIntakeState.ALTERNATING;
        };
    }

    private void handleClimbStateTransitions() {
        if (currentSystemState == CurrentSystemState.DISABLED || desiredClimbState == DesiredClimbState.DISABLED) {
            currentClimbState = CurrentClimbState.DISABLED;
            return;
        }

        currentClimbState = mapClimberCurrentState(climber.getCurrentState());
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
        applyClimbState(
            currentSystemState == CurrentSystemState.DISABLED
                ? DesiredClimbState.DISABLED
                : desiredClimbState
        );
    }

    private void handleDisabledState() {
        applyHomeOutputs();
    }

    private void handleHomeState() {
        applyHomeOutputs();
    }

    private void handleTrackingState() {
        applyTrackingOutputs();
    }

    private void handlePreparingForShotState() {
        applyDynamicShotOutputs(getPreparingForShotHopperSetpoint(lastStateBeforePreparingForShot));
    }

    private void handleReadyForShotState() {
        applyDynamicShotOutputs(HopperSetpoint.OFF);
    }

    private void handleShootingState() {        
        applyDynamicShotOutputs(HopperSetpoint.FEEDING);

        boolean shouldVisualizeShots = Constants.currentMode == Constants.Mode.SIM;
        if (!shouldVisualizeShots) {
            return;
        }

        double now = Timer.getTimestamp();
        double timeSinceShotStart = now - shotStartTime;
        if (timeSinceShotStart >= config.shotDurationSeconds) {
            if (!hasStartedShooting) {
                hasStartedShooting = true;
                lastBallVisualizedTime = now;
                new VisualizeShot(cachedShotData.exitVelocity());
            } else {
                double ballIntervalSeconds = 1.0 / config.ballsPerSecond;
                if (now - lastBallVisualizedTime >= ballIntervalSeconds) {
                    lastBallVisualizedTime = now;
                    new VisualizeShot(cachedShotData.exitVelocity());
                }
            }
        }
    }

    private void handleBumpState() {
        swerveDrive.setSnapTargetAngle(bumpSnapAngle);
        swerveDrive.setTranslationVelocityCapMaxVelocityMetersPerSec(config.bumpMaxVelocityMetersPerSec);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.SNAPPED);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.CAPPED);
        
        // Keep shooter in home position during bump
        shooter.setHoodSetpoint(HoodSetpoint.HOME);
        shooter.setTurretSetpoint(TurretSetpoint.HOME);
        shooter.setFlywheelSetpoint(FlywheelSetpoint.OFF);
        hopper.setSetpoint(HopperSetpoint.OFF);
    }

    private void applyHomeOutputs() {
        applyShooterAndHopperSetpoints(
            HoodSetpoint.HOME,
            TurretSetpoint.HOME,
            FlywheelSetpoint.OFF,
            HopperSetpoint.OFF
        );
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.NONE);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.NONE);
    }

    private void applyTrackingOutputs() {
        applyShooterAndHopperSetpoints(
            HoodSetpoint.DYNAMIC,
            TurretSetpoint.DYNAMIC,
            FlywheelSetpoint.OFF,
            HopperSetpoint.OFF
        );
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.NONE);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.NONE);
    }

    private void applyDynamicShotOutputs(HopperSetpoint hopperSetpoint) {
        applyShotMotionCaps();
        applyShooterAndHopperSetpoints(
            HoodSetpoint.DYNAMIC,
            TurretSetpoint.DYNAMIC,
            FlywheelSetpoint.DYNAMIC,
            hopperSetpoint
        );
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.CAPPED);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.CAPPED);
    }

    private void applyShotMotionCaps() {
        swerveDrive.setTranslationVelocityCapMaxVelocityMetersPerSec(config.maxTranslationalVelocityDuringShotMetersPerSec);
        swerveDrive.setOmegaVelocityCapMaxRadiansPerSec(config.maxAngularVelocityDuringShotRadPerSec);
    }

    private void applyShooterAndHopperSetpoints(
        HoodSetpoint hoodSetpoint,
        TurretSetpoint turretSetpoint,
        FlywheelSetpoint flywheelSetpoint,
        HopperSetpoint hopperSetpoint
    ) {
        shooter.setHoodSetpoint(hoodSetpoint);
        shooter.setTurretSetpoint(turretSetpoint);
        shooter.setFlywheelSetpoint(flywheelSetpoint);
        hopper.setSetpoint(hopperSetpoint);
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
            case REVERSING:
                intake.setSetpoint(IntakeSetpoint.OUTTAKING);
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

        boolean timerElapsed = now - lastAlternatingIntakeToggleTime >= config.alternatingIntakeToggleSeconds;
        boolean pivotSettled = intake.isPivotAtSetpoint();
        if (timerElapsed && pivotSettled) {
            lastAlternatingIntakeToggleTime = now;
            intake.setSetpoint(intake.isStowed() ? IntakeSetpoint.DEPLOYED : IntakeSetpoint.STOWED);
            return;
        }

        intake.setSetpoint(intake.isStowed() ? IntakeSetpoint.STOWED : IntakeSetpoint.DEPLOYED);
    }

    private void applyClimbState(DesiredClimbState climbState) {
        switch (climbState) {
            case DISABLED:
                climber.setDesiredState(Climber.DesiredState.DISABLED);
                break;
            case RETRACTED:
                climber.setDesiredState(Climber.DesiredState.RETRACTED);
                break;
            case EXTENDED:
                climber.setDesiredState(Climber.DesiredState.EXTENDED);
                break;
            case CLIMBED:
                climber.setDesiredState(Climber.DesiredState.CLIMBED);
                break;
        }
    }

    /**
     * Checks if all mechanisms are at their setpoints and ready to shoot.
     */
    private boolean isReadyForShot() {
        return cachedShotReadinessData.readyForShot();
    }

    private ShotReadinessData calculateShotReadinessData(ShotComputationContext context) {
        double actualHood = shooter.getHoodAngleRotations();
        double actualTurret = shooter.getTurretAngleRotations();
        double actualFlywheel = shooter.getFlywheelVelocityRotationsPerSec();

        // Use the commanded (clamped/resolved) shooter setpoints so impact math matches what hardware is tracking.
        double setpointHood = shooter.getHoodSetpointRotations();
        double setpointTurret = shooter.getTurretSetpointRotations();
        double setpointFlywheel = shooter.getFlywheelSetpointRPS();

        double hoodErrorDegrees = Math.abs((actualHood - setpointHood) * 360.0);
        double turretErrorRotations = MathUtil.inputModulus(actualTurret - setpointTurret, -0.5, 0.5);
        double turretErrorDegrees = Math.abs(turretErrorRotations * 360.0);
        double turretFieldRelativeErrorDegrees = calculateTurretFieldRelativeErrorDegrees(
            actualTurret,
            robotState.getEstimatedPose().getRotation(),
            cachedShotData.targetFieldYaw()
        );
        double flywheelErrorRps = Math.abs(actualFlywheel - setpointFlywheel);
        boolean hoodAtSetpoint = shooter.isHoodAtSetpoint();
        boolean turretAtSetpoint = shooter.isTurretAtSetpoint();
        boolean turretFieldRelativeAtSetpoint =
            turretFieldRelativeErrorDegrees <= shooter.getTurretAngleToleranceDegrees();
        boolean flywheelAtSetpoint = shooter.isFlywheelAtSetpoint();
        boolean swerveOmegaCapped = isSwerveOmegaCappedForShot();

        double effectiveDistance = cachedShotData.effectiveDistance();
        boolean shooterReady = hoodAtSetpoint
            && turretAtSetpoint
            && flywheelAtSetpoint
            && turretFieldRelativeAtSetpoint;
        double minShotDistance = context.minDistanceMeters();
        double maxShotDistance = context.maxDistanceMeters();
        boolean distanceInRange = isDistanceInRange(effectiveDistance, minShotDistance, maxShotDistance);
        boolean readyForShot = shooterReady
            && distanceInRange
            // && context.lineOfSightClear()
            && context.zoneAllowsTarget();

        return new ShotReadinessData(
            effectiveDistance,
            minShotDistance,
            maxShotDistance,
            hoodAtSetpoint,
            turretAtSetpoint,
            turretFieldRelativeAtSetpoint,
            flywheelAtSetpoint,
            swerveOmegaCapped,
            hoodErrorDegrees,
            turretErrorDegrees,
            turretFieldRelativeErrorDegrees,
            flywheelErrorRps,
            shooterReady,
            distanceInRange,
            context.lineOfSightClear(),
            context.zoneAllowsTarget(),
            readyForShot
        );
    }

    private void logShotReadinessData(ShotReadinessData data) {
        Logger.recordOutput("Superstructure/shooterReady", data.shooterReady());
        Logger.recordOutput("Superstructure/shotDistanceInRange", data.distanceInRange());
        Logger.recordOutput("Superstructure/effectiveShotDistanceMeters", data.effectiveDistanceMeters());
        Logger.recordOutput("Superstructure/minShotDistanceMeters", data.minShotDistanceMeters());
        Logger.recordOutput("Superstructure/maxShotDistanceMeters", data.maxShotDistanceMeters());
        Logger.recordOutput("Superstructure/hoodAtSetpoint", data.hoodAtSetpoint());
        Logger.recordOutput("Superstructure/turretAtSetpoint", data.turretAtSetpoint());
        Logger.recordOutput("Superstructure/turretFieldRelativeAtSetpoint", data.turretFieldRelativeAtSetpoint());
        Logger.recordOutput("Superstructure/flywheelAtSetpoint", data.flywheelAtSetpoint());
        Logger.recordOutput("Superstructure/swerveOmegaCapped", data.swerveOmegaCapped());
        Logger.recordOutput("Superstructure/hoodErrorDegrees", data.hoodErrorDegrees());
        Logger.recordOutput("Superstructure/turretErrorDegrees", data.turretErrorDegrees());
        Logger.recordOutput("Superstructure/turretFieldRelativeErrorDegrees", data.turretFieldRelativeErrorDegrees());
        Logger.recordOutput("Superstructure/flywheelErrorRps", data.flywheelErrorRps());
        Logger.recordOutput("Superstructure/targetLineOfSightClear", data.lineOfSightClear());
        Logger.recordOutput("Superstructure/targetZoneAllowed", data.zoneAllowsTarget());
    }

    static boolean didShotReachTargetHeight(Translation3d landingPosition, double targetHeightMeters) {
        return Math.abs(landingPosition.getZ() - targetHeightMeters) <= TARGET_HEIGHT_REACH_EPSILON_METERS;
    }

    static double calculateTurretFieldRelativeErrorDegrees(
        double turretAngleRotations,
        Rotation2d robotYaw,
        Rotation2d targetFieldYaw
    ) {
        double actualFieldYawDeg = robotYaw.plus(Rotation2d.fromRotations(turretAngleRotations)).getDegrees();
        return Math.abs(MathUtil.inputModulus(actualFieldYawDeg - targetFieldYaw.getDegrees(), -180.0, 180.0));
    }

    private boolean isSwerveOmegaCappedForShot() {
        SwerveDrive.CurrentOmegaOverrideState omegaState = swerveDrive.getCurrentOmegaOverrideState();
        return omegaState == SwerveDrive.CurrentOmegaOverrideState.CAPPED;
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

    static boolean isLastInRangeShotDataValid(
        double nowSeconds,
        double lastInRangeShotTimestampSeconds,
        double maxAgeSeconds
    ) {
        if (!Double.isFinite(lastInRangeShotTimestampSeconds)) {
            return false;
        }
        return nowSeconds - lastInRangeShotTimestampSeconds <= maxAgeSeconds;
    }

    /**
     * Updates swerve rotation bounds for shot-oriented states from live turret remaining travel.
     * This keeps robot yaw in a range that avoids turret wrap maneuvers.
     */
    private void updateSwerveRotationRangeForShotStates() {
        double accumulatedYawDeg = robotState.getAccumulatedYawDegrees();
        double turretCurrentDeg = shooter.getTurretAngleRotations() * 360.0;
        double turretMinDeg = shooter.getTurretMinAngleDeg();
        double turretMaxDeg = shooter.getTurretMaxAngleDeg();

        TurretRotationMargins turretMargins = calculateTurretRotationMargins(
            turretCurrentDeg,
            turretMinDeg,
            turretMaxDeg,
            config.turretRotationBufferDeg
        );
        Logger.recordOutput("Superstructure/shootingRange/turretCurrentDeg", turretCurrentDeg);
        Logger.recordOutput("Superstructure/shootingRange/turretMinDeg", turretMinDeg);
        Logger.recordOutput("Superstructure/shootingRange/turretMaxDeg", turretMaxDeg);
        Logger.recordOutput("Superstructure/shootingRange/marginTowardMinDeg", turretMargins.marginTowardMinDeg());
        Logger.recordOutput("Superstructure/shootingRange/marginTowardMaxDeg", turretMargins.marginTowardMaxDeg());
        Logger.recordOutput("Superstructure/shootingRange/marginsValid", turretMargins.valid());

        if (!turretMargins.valid()) {
            if (!hasWarnedInvalidTurretRotationMargins) {
                edu.wpi.first.wpilibj.DriverStation.reportWarning(
                    "Superstructure shooting rotation margins have no buffered travel on either side. Keeping prior ranged bounds.",
                    false
                );
                hasWarnedInvalidTurretRotationMargins = true;
            }
            return;
        }
        hasWarnedInvalidTurretRotationMargins = false;

        AccumulatedYawRotationRange accumulatedRange = calculateAccumulatedYawRotationRange(
            accumulatedYawDeg,
            turretMargins.marginTowardMinDeg(),
            turretMargins.marginTowardMaxDeg()
        );
        Logger.recordOutput("Superstructure/shootingRange/accumulatedYawDeg", accumulatedYawDeg);
        Logger.recordOutput("Superstructure/shootingRange/minAbsDeg", accumulatedRange.minAbsDeg());
        Logger.recordOutput("Superstructure/shootingRange/maxAbsDeg", accumulatedRange.maxAbsDeg());

        swerveDrive.setRotationRangeAccumulatedDegrees(
            accumulatedRange.minAbsDeg(),
            accumulatedRange.maxAbsDeg()
        );
    }

    static TurretRotationMargins calculateTurretRotationMargins(
        double turretCurrentDeg,
        double turretMinDeg,
        double turretMaxDeg,
        double turretBufferDeg
    ) {
        // One-sided headroom is still useful: clamp exhausted sides to zero so we can keep updating
        // the opposite yaw bound instead of freezing the entire ranged-rotation window.
        double marginTowardMinDeg = Math.max(0.0, turretCurrentDeg - turretMinDeg - turretBufferDeg);
        double marginTowardMaxDeg = Math.max(0.0, turretMaxDeg - turretCurrentDeg - turretBufferDeg);
        return new TurretRotationMargins(
            marginTowardMinDeg,
            marginTowardMaxDeg,
            marginTowardMinDeg > 0.0 || marginTowardMaxDeg > 0.0
        );
    }

    static AccumulatedYawRotationRange calculateAccumulatedYawRotationRange(
        double accumulatedYawDeg,
        double marginTowardMinDeg,
        double marginTowardMaxDeg
    ) {
        // targetFieldYaw = accumulatedYaw + turretCurrent
        // robotYaw bounds are [targetFieldYaw - turretMax, targetFieldYaw - turretMin]
        // equivalently [accumulatedYaw - marginTowardMax, accumulatedYaw + marginTowardMin]
        return new AccumulatedYawRotationRange(
            accumulatedYawDeg - marginTowardMaxDeg,
            accumulatedYawDeg + marginTowardMinDeg
        );
    }

    // Target suppliers for shooter - these use cached shot data
    // Target suppliers always provide values from shot calculator
    // Shooter decides when to use them based on its state
    private Rotation2d getTargetHoodAngle() {
        return cachedShotData.hoodPitch();
    }

    private Rotation2d getTargetTurretAngle() {
        return cachedShotData.targetFieldYaw();
    }

    private double getTargetTurretVelocityRotPerSec() {
        if (cachedShotData == null || cachedShotComputationContext == null) {
            return 0.0;
        }

        ShotKinematicsUtil.ShooterKinematics shooterKinematics = cachedShotComputationContext.shooterKinematics();
        double angularVelocityInFieldRadPerSec = ShotKinematicsUtil.calculateLineOfSightAngularVelocityRadPerSec(
            shooterKinematics.shooterPosition().toTranslation2d(),
            shooterKinematics.shooterVxField(),
            shooterKinematics.shooterVyField(),
            cachedShotData.compensatedTargetPosition()
        );

        Logger.recordOutput(
            "Superstructure/turretDesiredAngularVelocityFieldRadPerSec",
            angularVelocityInFieldRadPerSec
        );
        Logger.recordOutput(
            "Superstructure/turretDesiredAngularVelocityRotPerSec",
            angularVelocityInFieldRadPerSec / (2.0 * Math.PI)
        );
        return angularVelocityInFieldRadPerSec / (2.0 * Math.PI);
    }

    private double getTargetFlywheelRPS() {
        return cachedShotData.flywheelRPS();
    }

    private ShotData calculateShotData(ShotComputationContext context) {
        double lcomp = latencyCompensationSeconds.get();

        return ShotCalculator.calculate(
            context.targetLocation(),
            context.shooterKinematics().shooterPosition(),
            context.shooterKinematics().fieldRelativeSpeeds(),
            context.lerpTable(),
            lcomp,
            shooter.getFlywheelVelocityRotationsPerSec(),
            shooter::calculateShotExitVelocityMetersPerSec,
            rps -> shooter.calculateBackSpinRPM(rps) * 2.0 * Math.PI / 60.0,
            context.shooterKinematics().shooterOffsetFromRobotCenter(),
            context.shooterKinematics().robotHeading()
        );
    }

    private ShotComputationContext buildShotComputationContext() {
        ShotKinematicsUtil.ShooterKinematics shooterKinematics = ShotKinematicsUtil.calculateShooterKinematics(
            robotState.getEstimatedPose(),
            shooter.getShooterRelativePose(),
            robotState.getFieldRelativeSpeeds()
        );
        RobotFieldZone robotFieldZone = getRobotFieldZone();
        TargetState zoneResolvedTarget = resolveTargetForZoneConstraints(desiredTargetState, robotFieldZone);
        boolean zoneAllowsTarget = isTargetAllowedInZone(zoneResolvedTarget, robotFieldZone);
        Translation3d targetLocation = getFieldTargetLocation(zoneResolvedTarget);
        boolean passingTarget = zoneResolvedTarget.isPassTarget();
        Translation2d shooterPosition = shooterKinematics.shooterPosition().toTranslation2d();
        Translation2d targetPosition = targetLocation.toTranslation2d();
        RectangleZone[] hubLineOfSightBlockers = buildMirroredRectangularBlockers(ZoneConstants.Tower.EXCLUSION);
        RectangleZone[] passLineOfSightBlockers = buildMirroredRectangularBlockers(
            ZoneConstants.Hub.EXCLUSION,
            ZoneConstants.Tower.EXCLUSION
        );
        boolean lineOfSightClear = passingTarget
            ? hasClearLineOfSightWithRectangularBlockers(
                shooterPosition,
                targetPosition,
                config.passHubBlockerRadiusMeters,
                passLineOfSightBlockers
            )
            : hasClearLineOfSightWithRectangularBlockers(
                shooterPosition,
                targetPosition,
                config.passHubBlockerRadiusMeters,
                hubLineOfSightBlockers
            );

        currentTargetState = zoneResolvedTarget;

        InterpolatingMatrixTreeMap<Double, N3, N1> lerpTable = passingTarget
            ? shooter.getPassLerpTable()
            : shooter.getLerpTable();
        double minDistance = passingTarget
            ? shooter.getMinPassDistFromShooterMeters()
            : shooter.getMinShotDistFromShooterMeters();
        double maxDistance = passingTarget
            ? shooter.getMaxPassDistFromShooterMeters()
            : shooter.getMaxShotDistFromShooterMeters();

        Logger.recordOutput("Superstructure/targetStateDesired", desiredTargetState.toString());
        Logger.recordOutput("Superstructure/targetStateCurrent", currentTargetState.toString());
        Logger.recordOutput("Superstructure/robotFieldZone", robotFieldZone.toString());
        Logger.recordOutput("Superstructure/targetLocation", targetLocation);
        Logger.recordOutput("Superstructure/passingTarget", passingTarget);

        return new ShotComputationContext(
            currentTargetState,
            targetLocation,
            shooterKinematics,
            lerpTable,
            minDistance,
            maxDistance,
            passingTarget,
            lineOfSightClear,
            zoneAllowsTarget
        );
    }

    private RobotFieldZone getRobotFieldZone() {
        if (ZoneUtil.isPoseInAnyZone(robotState.getEstimatedPose(), ZoneConstants.Alliance.COMPOSITE, true)) {
            return RobotFieldZone.CURRENT_ALLIANCE;
        }
        if (ZoneUtil.isPoseInAnyZone(robotState.getEstimatedPose(), ZoneConstants.Neutral.COMPOSITE, true)) {
            return RobotFieldZone.NEUTRAL;
        }
        if (ZoneUtil.isPoseInOpposingAllianceZone(robotState.getEstimatedPose(), ZoneConstants.OpposingAlliance.COMPOSITE)) {
            return RobotFieldZone.OPPOSING_ALLIANCE;
        }
        return RobotFieldZone.UNKNOWN;
    }

    static TargetState resolveTargetForZoneConstraints(TargetState desiredTargetState, RobotFieldZone robotFieldZone) {
        // Do not force-retarget; preserve the user-selected target.
        return desiredTargetState;
    }

    static boolean isTargetAllowedInZone(TargetState targetState, RobotFieldZone robotFieldZone) {
        if (targetState == TargetState.HUB) {
            return robotFieldZone == RobotFieldZone.CURRENT_ALLIANCE;
        }
        // Passing is legal from any zone.
        return true;
    }

    static boolean hasClearLineOfSightWithRectangularBlockers(
        Translation2d shooterPosition,
        Translation2d targetPosition,
        double blockerRadiusMeters,
        RectangleZone... blockerZones
    ) {
        for (RectangleZone blockerZone : blockerZones) {
            if (!ZoneUtil.hasLineOfSightWithRectangularBlocker(
                shooterPosition,
                targetPosition,
                blockerZone,
                blockerRadiusMeters,
                false
            )) {
                return false;
            }
        }

        return true;
    }

    static Optional<TargetState> selectClosestLineOfSightTarget(
        Translation2d shooterPosition,
        TargetCandidate... candidates
    ) {
        TargetState closestLineOfSightTarget = null;
        double closestDistanceMeters = Double.POSITIVE_INFINITY;

        for (TargetCandidate candidate : candidates) {
            if (!candidate.lineOfSightClear()) {
                continue;
            }

            double candidateDistanceMeters = shooterPosition.getDistance(candidate.targetPosition());
            if (candidateDistanceMeters < closestDistanceMeters) {
                closestDistanceMeters = candidateDistanceMeters;
                closestLineOfSightTarget = candidate.targetState();
            }
        }

        return Optional.ofNullable(closestLineOfSightTarget);
    }

    private static RectangleZone flipRectangleZone(RectangleZone zone) {
        return new RectangleZone(
            zone.name() + "_flipped",
            FlippingUtil.flipFieldPosition(zone.cornerA()),
            FlippingUtil.flipFieldPosition(zone.cornerB())
        );
    }

    private static RectangleZone[] buildMirroredRectangularBlockers(RectangleZone... baseZones) {
        RectangleZone[] blockerZones = new RectangleZone[baseZones.length * 2];
        for (int i = 0; i < baseZones.length; i++) {
            RectangleZone baseZone = baseZones[i];
            blockerZones[i * 2] = baseZone;
            blockerZones[i * 2 + 1] = flipRectangleZone(baseZone);
        }
        return blockerZones;
    }

    private Translation3d getFieldTargetLocation(TargetState targetState) {
        Translation3d targetLocation = switch (targetState) {
            case HUB -> FieldConstants.Hub.hubCenter;
            case PASS_ALLIANCE_TOP -> FieldConstants.Passing.allianceTop;
            case PASS_ALLIANCE_BOTTOM -> FieldConstants.Passing.allianceBottom;
            case PASS_NEUTRAL_TOP -> FieldConstants.Passing.neutralTop;
            case PASS_NEUTRAL_BOTTOM -> FieldConstants.Passing.neutralBottom;
        };
        if (Constants.shouldFlipPath()) {
            Translation2d fieldPosition = FlippingUtil.flipFieldPosition(targetLocation.toTranslation2d());
            targetLocation = new Translation3d(fieldPosition.getX(), fieldPosition.getY(), targetLocation.getZ());
        }
        return targetLocation;
    }

    public Optional<TargetState> getClosestLineOfSightAlliancePassTarget() {
        ShotKinematicsUtil.ShooterKinematics shooterKinematics = ShotKinematicsUtil.calculateShooterKinematics(
            robotState.getEstimatedPose(),
            shooter.getShooterRelativePose(),
            robotState.getFieldRelativeSpeeds()
        );
        Translation2d shooterPosition = shooterKinematics.shooterPosition().toTranslation2d();
        RectangleZone[] passLineOfSightBlockers = buildMirroredRectangularBlockers(
            ZoneConstants.Hub.EXCLUSION,
            ZoneConstants.Tower.EXCLUSION
        );

        ArrayList<TargetCandidate> candidates = new ArrayList<>();
        for (TargetState targetState : TargetState.values()) {
            if (!targetState.isAlliancePassTarget()) {
                continue;
            }
            Translation2d targetPosition = getFieldTargetLocation(targetState).toTranslation2d();
            boolean lineOfSightClear = hasClearLineOfSightWithRectangularBlockers(
                shooterPosition,
                targetPosition,
                config.passHubBlockerRadiusMeters,
                passLineOfSightBlockers
            );
            candidates.add(new TargetCandidate(targetState, targetPosition, lineOfSightClear));
        }

        Optional<TargetState> selectedTarget = selectClosestLineOfSightTarget(
            shooterPosition,
            candidates.toArray(TargetCandidate[]::new)
        );
        Logger.recordOutput("Superstructure/closestAlliancePassTargetFound", selectedTarget.isPresent());
        Logger.recordOutput(
            "Superstructure/closestAlliancePassTarget",
            selectedTarget.map(Enum::name).orElse("NONE")
        );
        return selectedTarget;
    }

    private static CurrentClimbState mapClimberCurrentState(Climber.CurrentState climberCurrentState) {
        return switch (climberCurrentState) {
            case DISABLED -> CurrentClimbState.DISABLED;
            case RETRACTED -> CurrentClimbState.RETRACTED;
            case EXTENDED -> CurrentClimbState.EXTENDED;
            case CLIMBED -> CurrentClimbState.CLIMBED;
        };
    }

    static HopperSetpoint getPreparingForShotHopperSetpoint(CurrentSystemState lastStateBeforePreparingForShot) {
        return lastStateBeforePreparingForShot == CurrentSystemState.SHOOTING
            ? HopperSetpoint.FEEDING_IDLE
            : HopperSetpoint.OFF;
    }

    // Public interface
    public void setDesiredSystemState(DesiredSystemState desiredSystemState) {
        this.desiredSystemState = desiredSystemState;
    }

    public void setDesiredIntakeState(DesiredIntakeState desiredIntakeState) {
        this.desiredIntakeState = desiredIntakeState;
    }

    public void setDesiredClimbState(DesiredClimbState desiredClimbState) {
        this.desiredClimbState = desiredClimbState;
    }

    public void setDesiredTargetState(TargetState desiredTargetState) {
        this.desiredTargetState = desiredTargetState;
    }

    @AutoLogOutput(key = "Superstructure/currentSystemState")
    public CurrentSystemState getCurrentSystemState() {
        return currentSystemState;
    }

    @AutoLogOutput(key = "Superstructure/lastStateBeforePreparingForShot")
    public CurrentSystemState getLastStateBeforePreparingForShot() {
        return lastStateBeforePreparingForShot;
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

    @AutoLogOutput(key = "Superstructure/currentClimbState")
    public CurrentClimbState getCurrentClimbState() {
        return currentClimbState;
    }

    @AutoLogOutput(key = "Superstructure/desiredClimbState")
    public DesiredClimbState getDesiredClimbState() {
        return desiredClimbState;
    }

    @AutoLogOutput(key = "Superstructure/currentTargetState")
    public TargetState getCurrentTargetState() {
        return currentTargetState;
    }

    @AutoLogOutput(key = "Superstructure/desiredTargetState")
    public TargetState getDesiredTargetState() {
        return desiredTargetState;
    }

    public Translation3d getCurrentFieldTargetLocation() {
        return getFieldTargetLocation(currentTargetState);
    }

    public InterpolatingMatrixTreeMap<Double, N3, N1> getCurrentTargetLerpTable() {
        return currentTargetState.isPassTarget() ? shooter.getPassLerpTable() : shooter.getLerpTable();
    }

    @AutoLogOutput(key = "Superstructure/isClimbExtended")
    public boolean isClimbExtended() {
        return currentClimbState == CurrentClimbState.EXTENDED;
    }

    @AutoLogOutput(key = "Superstructure/isClimbAtDesiredState")
    public boolean isClimbAtDesiredState() {
        return switch (desiredClimbState) {
            case DISABLED -> currentClimbState == CurrentClimbState.DISABLED;
            case RETRACTED -> currentClimbState == CurrentClimbState.RETRACTED;
            case EXTENDED -> currentClimbState == CurrentClimbState.EXTENDED;
            case CLIMBED -> currentClimbState == CurrentClimbState.CLIMBED;
        };
    }
}
