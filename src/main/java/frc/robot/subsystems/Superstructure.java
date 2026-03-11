package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
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
import frc.robot.lib.util.ballistics.BallisticsPhysics;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.util.ConfigLoader;
import frc.robot.lib.util.ShotKinematicsUtil;
import frc.robot.lib.util.ShotCalculator;
import frc.robot.lib.util.ShotCalculator.ShotData;
import frc.robot.lib.util.ZoneUtil;
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

    public enum TargetState {
        HUB,
        PASS_ALLIANCE_TOP,
        PASS_ALLIANCE_CENTER,
        PASS_ALLIANCE_BOTTOM,
        PASS_NEUTRAL_TOP,
        PASS_NEUTRAL_CENTER,
        PASS_NEUTRAL_BOTTOM
    }

    enum RobotFieldZone {
        CURRENT_ALLIANCE,
        NEUTRAL,
        OPPOSING_ALLIANCE,
        UNKNOWN
    }

    private DesiredSystemState desiredSystemState = DesiredSystemState.DISABLED;
    private CurrentSystemState currentSystemState = CurrentSystemState.DISABLED;
    private DesiredIntakeState desiredIntakeState = DesiredIntakeState.STOWED;
    private CurrentIntakeState currentIntakeState = CurrentIntakeState.DISABLED;
    private TargetState desiredTargetState = TargetState.HUB;
    private TargetState currentTargetState = TargetState.HUB;

    private final Shooter shooter = Shooter.getInstance();
    private final Hopper hopper = Hopper.getInstance();
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Intake intake = Intake.getInstance();
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
        new Translation3d(),
        new Translation3d(),
        Double.POSITIVE_INFINITY,
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
        Translation3d actualLanding,
        Translation3d setpointLanding,
        double impactErrorMeters,
        double effectiveDistanceMeters,
        double minShotDistanceMeters,
        double maxShotDistanceMeters,
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

    private Superstructure() {
        config = ConfigLoader.load("superstructure", SuperstructureConfig.class);
        bumpSnapAngle = Rotation2d.fromDegrees(config.bumpSnapAngleDegrees);

        // Set up suppliers for the shooter - these provide dynamic setpoints based on shot calculation
        shooter.setHoodAngleSupplier(this::getTargetHoodAngle);
        shooter.setTurretAngleSupplier(this::getTargetTurretAngle);
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
        Logger.recordOutput("Superstructure/lastInRangeShotMaxAgeSeconds", config.lastInRangeShotMaxAgeSeconds);
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
    }

    private void handleSystemStateTransitions() {
        if (currentSystemState == CurrentSystemState.SHOOTING
            && Timer.getTimestamp() - shotStartTime < config.shotDurationSeconds) {
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
        applyHomeOutputs();
    }

    private void handleHomeState() {
        applyHomeOutputs();
    }

    private void handleTrackingState() {
        applyTrackingOutputs();
    }

    private void handlePreparingForShotState() {
        applyDynamicShotOutputs(HopperSetpoint.OFF);
    }

    private void handleReadyForShotState() {
        applyDynamicShotOutputs(HopperSetpoint.OFF);
    }

    private void handleShootingState() {        
        applyDynamicShotOutputs(HopperSetpoint.FEEDING);

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
        updateSwerveRotationRangeForShotStates();
        applyShotMotionCaps();
        applyShooterAndHopperSetpoints(
            HoodSetpoint.DYNAMIC,
            TurretSetpoint.DYNAMIC,
            FlywheelSetpoint.DYNAMIC,
            hopperSetpoint
        );
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.RANGED_ROTATION_CAPPED);
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

    /**
     * Checks if all mechanisms are at their setpoints and ready to shoot.
     * Uses shot impact error and lerp table distance bounds as readiness metrics.
     */
    private boolean isReadyForShot() {
        return cachedShotReadinessData.readyForShot();
    }

    private ShotReadinessData calculateShotReadinessData(ShotComputationContext context) {
        double actualHood = shooter.getHoodAngleRotations();
        double actualTurret = shooter.getTurretAngleRotations();
        double actualFlywheel = shooter.getFlywheelVelocityRotationsPerSec();

        double setpointHood = getTargetHoodAngle().getRotations();
        double setpointTurret = getTargetTurretAngle().getRotations();
        double setpointFlywheel = getTargetFlywheelRPS();

        double effectiveDistance = cachedShotData.effectiveDistance();
        double shooterHeight = context.shooterKinematics().shooterPosition().getZ();
        double targetHeight = context.targetLocation().getZ();

        double actualExitVelocity = cachedShotData.exitVelocity();
        double actualSpinRateRadPerSec = ShotCalculator.calculateSpinRateRadPerSec(
            effectiveDistance,
            context.lerpTable(),
            actualFlywheel,
            shooter::calculateShotExitVelocityMetersPerSec,
            rps -> shooter.calculateBackSpinRPM(rps) * 2.0 * Math.PI / 60.0,
            shooterHeight,
            targetHeight,
            !context.passingTarget()
        );
        double setpointExitVelocity = ShotCalculator.calculateExitVelocityMetersPerSec(
            effectiveDistance,
            context.lerpTable(),
            setpointFlywheel,
            shooter::calculateShotExitVelocityMetersPerSec,
            rps -> shooter.calculateBackSpinRPM(rps) * 2.0 * Math.PI / 60.0,
            shooterHeight,
            targetHeight,
            !context.passingTarget()
        );
        double setpointSpinRateRadPerSec = ShotCalculator.calculateSpinRateRadPerSec(
            effectiveDistance,
            context.lerpTable(),
            setpointFlywheel,
            shooter::calculateShotExitVelocityMetersPerSec,
            rps -> shooter.calculateBackSpinRPM(rps) * 2.0 * Math.PI / 60.0,
            shooterHeight,
            targetHeight,
            !context.passingTarget()
        );

        Translation3d actualLanding = simulateShotLanding(
            context,
            actualHood,
            actualTurret,
            actualExitVelocity,
            actualSpinRateRadPerSec
        );
        Translation3d setpointLanding = simulateShotLanding(
            context,
            setpointHood,
            setpointTurret,
            setpointExitVelocity,
            setpointSpinRateRadPerSec
        );
        double impactErrorMeters = actualLanding.toTranslation2d()
            .getDistance(setpointLanding.toTranslation2d());
        boolean shooterReady = impactErrorMeters <= config.shotImpactToleranceMeters;
        double minShotDistance = context.minDistanceMeters();
        double maxShotDistance = context.maxDistanceMeters();
        boolean distanceInRange = isDistanceInRange(effectiveDistance, minShotDistance, maxShotDistance);
        boolean readyForShot = isShotReady(
            impactErrorMeters,
            config.shotImpactToleranceMeters,
            effectiveDistance,
            minShotDistance,
            maxShotDistance
        ) && context.lineOfSightClear() && context.zoneAllowsTarget();

        return new ShotReadinessData(
            actualLanding,
            setpointLanding,
            impactErrorMeters,
            effectiveDistance,
            minShotDistance,
            maxShotDistance,
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
        Logger.recordOutput("Superstructure/actualLanding", data.actualLanding());
        Logger.recordOutput("Superstructure/setpointLanding", data.setpointLanding());
        Logger.recordOutput("Superstructure/impactErrorMeters", data.impactErrorMeters());
        Logger.recordOutput("Superstructure/targetLineOfSightClear", data.lineOfSightClear());
        Logger.recordOutput("Superstructure/targetZoneAllowed", data.zoneAllowsTarget());
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
        return cachedShotData.targetFieldYaw().minus(robotState.getEstimatedPose().getRotation());
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
            context.shooterKinematics().robotHeading(),
            !context.passingTarget()
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
        boolean passingTarget = isPassingTarget(zoneResolvedTarget);
        boolean lineOfSightClear = !passingTarget
            || isPassLineOfSightClear(
                shooterKinematics.shooterPosition().toTranslation2d(),
                targetLocation.toTranslation2d(),
                ZoneConstants.Hub.EXCLUSION,
                flipRectangleZone(ZoneConstants.Hub.EXCLUSION),
                config.passHubBlockerRadiusMeters
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
        Logger.recordOutput("Superstructure/targetZoneAllowed", zoneAllowsTarget);
        Logger.recordOutput("Superstructure/passLineOfSightClear", lineOfSightClear);

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

    static boolean isPassingTarget(TargetState targetState) {
        return targetState != TargetState.HUB;
    }

    static boolean isPassLineOfSightClear(
        Translation2d shooterPosition,
        Translation2d passingTargetPosition,
        RectangleZone hubZone,
        RectangleZone flippedHubZone,
        double hubBlockerRadiusMeters
    ) {
        return ZoneUtil.hasLineOfSightWithRectangularBlocker(
                shooterPosition,
                passingTargetPosition,
                hubZone,
                hubBlockerRadiusMeters,
                false
            )
            && ZoneUtil.hasLineOfSightWithRectangularBlocker(
                shooterPosition,
                passingTargetPosition,
                flippedHubZone,
                hubBlockerRadiusMeters,
                false
            );
    }

    private static RectangleZone flipRectangleZone(RectangleZone zone) {
        return new RectangleZone(
            zone.name() + "_flipped",
            FlippingUtil.flipFieldPosition(zone.cornerA()),
            FlippingUtil.flipFieldPosition(zone.cornerB())
        );
    }

    private Translation3d getFieldTargetLocation(TargetState targetState) {
        Translation3d targetLocation = switch (targetState) {
            case HUB -> FieldConstants.Hub.hubCenter;
            case PASS_ALLIANCE_TOP -> FieldConstants.Passing.allianceTop;
            case PASS_ALLIANCE_CENTER -> FieldConstants.Passing.allianceCenter;
            case PASS_ALLIANCE_BOTTOM -> FieldConstants.Passing.allianceBottom;
            case PASS_NEUTRAL_TOP -> FieldConstants.Passing.neutralTop;
            case PASS_NEUTRAL_CENTER -> FieldConstants.Passing.neutralCenter;
            case PASS_NEUTRAL_BOTTOM -> FieldConstants.Passing.neutralBottom;
        };
        if (Constants.shouldFlipPath()) {
            Translation2d fieldPosition = FlippingUtil.flipFieldPosition(targetLocation.toTranslation2d());
            targetLocation = new Translation3d(fieldPosition.getX(), fieldPosition.getY(), targetLocation.getZ());
        }
        return targetLocation;
    }

    private Translation3d simulateShotLanding(
        ShotComputationContext context,
        double hoodAngleRotations,
        double turretAngleRotations,
        double exitVelocity,
        double spinRateRadPerSec
    ) {
        Translation3d shooterPosition = context.shooterKinematics().shooterPosition();

        double pitch = hoodAngleRotations * 2.0 * Math.PI;
        double turretYaw = turretAngleRotations * 2.0 * Math.PI;
        Rotation2d robotHeading = context.shooterKinematics().robotHeading();
        double fieldYaw = turretYaw + robotHeading.getRadians();

        double vHorizontal = exitVelocity * Math.cos(pitch);
        double vx = vHorizontal * Math.cos(fieldYaw) + context.shooterKinematics().shooterVxField();
        double vy = vHorizontal * Math.sin(fieldYaw) + context.shooterKinematics().shooterVyField();
        double vz = exitVelocity * Math.sin(pitch);

        return BallisticsPhysics.simulateToHeight3D(
            shooterPosition,
            vx,
            vy,
            vz,
            spinRateRadPerSec,
            context.targetLocation().getZ(),
            config.ballisticsSimulationStepSeconds
        );
    }

    // Public interface
    public void setDesiredSystemState(DesiredSystemState desiredSystemState) {
        this.desiredSystemState = desiredSystemState;
    }

    public void setDesiredIntakeState(DesiredIntakeState desiredIntakeState) {
        this.desiredIntakeState = desiredIntakeState;
    }

    public void setDesiredTargetState(TargetState desiredTargetState) {
        this.desiredTargetState = desiredTargetState;
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
        return isPassingTarget(currentTargetState) ? shooter.getPassLerpTable() : shooter.getLerpTable();
    }
}
