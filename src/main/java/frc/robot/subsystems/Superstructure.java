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
import edu.wpi.first.math.numbers.N2;
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
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.Kicker.KickerSetpoint;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.FlywheelSetpoint;
import frc.robot.subsystems.shooter.Shooter.HoodSetpoint;
import frc.robot.subsystems.shooter.Shooter.TurretSetpoint;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance;
    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    public enum CurrentState {
        DISABLED,
        HOME,
        TRACKING,
        PREPARING_FOR_SHOT,
        READY_FOR_SHOT,
        SHOOTING,
        BUMP
    }

    public enum DesiredState {
        DISABLED,
        HOME,
        TRACKING,
        READY_FOR_SHOT,
        SHOOTING,
        BUMP
    }

    private CurrentState currentState = CurrentState.DISABLED;
    private DesiredState desiredState = DesiredState.DISABLED;

    private final Shooter shooter = Shooter.getInstance();
    private final Kicker kicker = Kicker.getInstance();
    private final Hopper hopper = Hopper.getInstance();
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final RobotState robotState = RobotState.getInstance();

    private double lastShotTime = 0;
    private static final double SHOT_DURATION_SECONDS = .3; // Time to complete one shot
    private static final double BALLS_PER_SECOND = 12.0; // Balls per second to visualize

    // Margin for swerve rotation range (degrees)
    private static final double SWERVE_ROTATION_MARGIN_DEG = 20.0;
    private static final double MAX_TRANSLATIONAL_VELOCITY_DURING_SHOT_METERS_PER_SEC = 2.5;
    private static final double SHOT_IMPACT_TOLERANCE_METERS = 0.3;
    private static final double BUMP_MAX_VELOCITY_METERS_PER_SEC = 1.8;
    private static final Rotation2d BUMP_SNAP_ANGLE = Rotation2d.fromDegrees(45);
    private LoggedNetworkNumber latencyCompensationSeconds = new LoggedNetworkNumber("Shooter/latencyCompSec");
    private double kickerEngagedTime = 0;
    private double lastBallVisualizedTime = 0;
    private boolean hasStartedShooting = false;

    private Superstructure() {
        // Set up suppliers for the shooter - these provide dynamic setpoints based on shot calculation
        shooter.setHoodAngleSupplier(this::getTargetHoodAngle);
        shooter.setTurretAngleSupplier(this::getTargetTurretAngle);
        shooter.setFlywheelRPSSupplier(this::getTargetFlywheelRPS);
    }

    @Override
    public void periodic() {
        handleStateTransitions();
        handleCurrentState();
    }

    /**
     * Determines the next measured state based on the desired state.
     * Handles transitions between states with proper validation.
     */
    private void handleStateTransitions() {
        switch (desiredState) {
            case DISABLED:
                currentState = CurrentState.DISABLED;
                break;

            case HOME:
                if (currentState == CurrentState.SHOOTING && Timer.getTimestamp() - lastShotTime < SHOT_DURATION_SECONDS) {
                    break;
                }

                currentState = CurrentState.HOME;
                break;

            case TRACKING:
                if (currentState == CurrentState.SHOOTING && Timer.getTimestamp() - lastShotTime < SHOT_DURATION_SECONDS) {
                    break;
                }

                currentState = CurrentState.TRACKING;
                break;

            case READY_FOR_SHOT:
                // READY_FOR_SHOT requires mechanisms to be at setpoints
                // Otherwise we're in PREPARING_FOR_SHOT
                if (isReadyForShot()) {
                    currentState = CurrentState.READY_FOR_SHOT;
                    lastShotTime = Timer.getTimestamp();
                } else {
                    currentState = CurrentState.PREPARING_FOR_SHOT;
                }
                break;

            case SHOOTING:
                // SHOOTING only allowed when we're in READY_FOR_SHOT
                if (currentState == CurrentState.READY_FOR_SHOT) {
                    currentState = CurrentState.SHOOTING;
                    kickerEngagedTime = Timer.getTimestamp();
                    lastShotTime = kickerEngagedTime;
                    hasStartedShooting = false;
                } else {
                    // Not ready yet, go to preparing
                    if (isReadyForShot()) {
                        currentState = CurrentState.SHOOTING;
                        lastShotTime = Timer.getTimestamp();
                    } else {
                        currentState = CurrentState.PREPARING_FOR_SHOT;
                    }
                }
                break;

            case BUMP:
                currentState = CurrentState.BUMP;
                break;
        }

        latencyCompensationSeconds.setDefault(shooter.getLatencyCompensationSeconds());
    }

    /**
     * Executes behavior for the current state and sets subsystem states.
     */
    private void handleCurrentState() {
        switch (currentState) {
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
    }

    private void handleDisabledState() {
        shooter.setHoodSetpoint(HoodSetpoint.HOME);
        shooter.setTurretSetpoint(TurretSetpoint.HOME);
        shooter.setFlywheelSetpoint(FlywheelSetpoint.OFF);
        kicker.setSetpoint(KickerSetpoint.OFF);
        hopper.setSetpoint(HopperSetpoint.OFF);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.NONE);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.NONE);
    }

    private void handleHomeState() {
        shooter.setHoodSetpoint(HoodSetpoint.HOME);
        shooter.setTurretSetpoint(TurretSetpoint.HOME);
        shooter.setFlywheelSetpoint(FlywheelSetpoint.OFF);
        kicker.setSetpoint(KickerSetpoint.OFF);
        hopper.setSetpoint(HopperSetpoint.OFF);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.NONE);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.NONE);
    }

    private void handleTrackingState() {
        shooter.setHoodSetpoint(HoodSetpoint.DYNAMIC);
        shooter.setTurretSetpoint(TurretSetpoint.DYNAMIC);
        shooter.setFlywheelSetpoint(FlywheelSetpoint.OFF);
        kicker.setSetpoint(KickerSetpoint.OFF);
        hopper.setSetpoint(HopperSetpoint.OFF);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.NONE);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.NONE);
    }

    private void handlePreparingForShotState() {
        updateSwerveRotationRange();
        swerveDrive.setVelocityCapMaxVelocityMetersPerSec(MAX_TRANSLATIONAL_VELOCITY_DURING_SHOT_METERS_PER_SEC);


        shooter.setHoodSetpoint(HoodSetpoint.DYNAMIC);
        shooter.setTurretSetpoint(TurretSetpoint.DYNAMIC);
        shooter.setFlywheelSetpoint(FlywheelSetpoint.DYNAMIC);
        kicker.setSetpoint(KickerSetpoint.FEEDING);
        hopper.setSetpoint(HopperSetpoint.OFF);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.RANGED_ROTATION);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.CAPPED);
    }

    private void handleReadyForShotState() {
        updateSwerveRotationRange();
        swerveDrive.setVelocityCapMaxVelocityMetersPerSec(MAX_TRANSLATIONAL_VELOCITY_DURING_SHOT_METERS_PER_SEC);

        shooter.setHoodSetpoint(HoodSetpoint.DYNAMIC);
        shooter.setTurretSetpoint(TurretSetpoint.DYNAMIC);
        shooter.setFlywheelSetpoint(FlywheelSetpoint.DYNAMIC);
        kicker.setSetpoint(KickerSetpoint.FEEDING);
        hopper.setSetpoint(HopperSetpoint.OFF);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.RANGED_ROTATION);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.CAPPED);
    }

    private void handleShootingState() {        
        updateSwerveRotationRange();
        swerveDrive.setVelocityCapMaxVelocityMetersPerSec(MAX_TRANSLATIONAL_VELOCITY_DURING_SHOT_METERS_PER_SEC);

        shooter.setHoodSetpoint(HoodSetpoint.DYNAMIC);
        shooter.setTurretSetpoint(TurretSetpoint.DYNAMIC);
        shooter.setFlywheelSetpoint(FlywheelSetpoint.DYNAMIC);
        kicker.setSetpoint(KickerSetpoint.KICKING);
        hopper.setSetpoint(HopperSetpoint.FEEDING);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.RANGED_ROTATION);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.CAPPED);

        double now = Timer.getTimestamp();
        double timeSinceKickerEngaged = now - kickerEngagedTime;
        if (timeSinceKickerEngaged >= SHOT_DURATION_SECONDS) {
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
        swerveDrive.setVelocityCapMaxVelocityMetersPerSec(BUMP_MAX_VELOCITY_METERS_PER_SEC);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.RANGED_ROTATION);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.CAPPED);
        
        // Keep shooter in home position during bump
        shooter.setHoodSetpoint(HoodSetpoint.HOME);
        shooter.setTurretSetpoint(TurretSetpoint.HOME);
        shooter.setFlywheelSetpoint(FlywheelSetpoint.OFF);
        kicker.setSetpoint(KickerSetpoint.OFF);
    }

    /**
     * Checks if all mechanisms are at their setpoints and ready to shoot.
     * Requires swerve to be in nominal ranged rotation, shooter to be ready,
     * kicker to be ready, and robot to be within valid shooting distance.
     */
    private boolean isReadyForShot() {
        // Check if swerve is in a nominal ranged rotation state
        boolean swerveReady = 
            swerveDrive.getCurrentOmegaOverrideState() == SwerveDrive.CurrentOmegaOverrideState.RANGED_NOMINAL &&
            Math.hypot(
                robotState.getFieldRelativeSpeeds().vxMetersPerSecond,
                robotState.getFieldRelativeSpeeds().vyMetersPerSecond
            ) < MAX_TRANSLATIONAL_VELOCITY_DURING_SHOT_METERS_PER_SEC;
        Logger.recordOutput("Superstructure/swerveReady", swerveReady);
        
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
        
        // Check if kicker is ready (feeding and at setpoint)
        boolean kickerReady = kicker.isKickerAtSetpoint();
        Logger.recordOutput("Superstructure/kickerReady", kickerReady);
        
        // Check if within valid shooting distance
        ShotData shotData = calculateShotData();
        double distance = shotData.effectiveDistance();
        boolean withinShotDistance = distance >= shooter.getMinShotDistFromShooterMeters() 
                                  && distance <= shooter.getMaxShotDistFromShooterMeters();
        
        Logger.recordOutput("Superstructure/withinShotDistance", withinShotDistance);

        return swerveReady && shooterReady && kickerReady && withinShotDistance;
    }

    /**
     * Updates the swerve rotation range based on turret limits and target shot angle.
     * This ensures the robot heading keeps the turret within its physical limits.
     */
    private void updateSwerveRotationRange() {
        ShotData shotData = calculateShotData();
        Logger.recordOutput("Superstructure/shotData", shotData);
        Rotation2d targetFieldYaw = shotData.targetFieldYaw();
        
        // Get turret physical limits
        double turretMinDeg = shooter.getTurretMinAngleDeg();
        double turretMaxDeg = shooter.getTurretMaxAngleDeg();
        
        // Calculate robot rotation range
        // Robot rotation = target field yaw - turret angle
        // So: turret angle = target field yaw - robot rotation
        // For turret to be within [turretMin, turretMax]:
        // Robot rotation must be within [targetYaw - turretMax, targetYaw - turretMin]
        // Add margin to ensure we stay well within bounds
        Rotation2d robotRotationMin = targetFieldYaw.minus(Rotation2d.fromDegrees(turretMaxDeg - SWERVE_ROTATION_MARGIN_DEG));
        Rotation2d robotRotationMax = targetFieldYaw.minus(Rotation2d.fromDegrees(turretMinDeg + SWERVE_ROTATION_MARGIN_DEG));
        
        swerveDrive.setRotationRange(robotRotationMin, robotRotationMax);
    }

    // Target suppliers for shooter - these calculate dynamic setpoints
    // Target suppliers always provide values from shot calculator
    // Shooter decides when to use them based on its state
    private Rotation2d getTargetHoodAngle() {
        ShotData shotData = calculateShotData();
        Logger.recordOutput("Superstructure/shotData", shotData);
        return shotData.hoodPitch();
    }

    private Rotation2d getTargetTurretAngle() {
        ShotData shotData = calculateShotData();
        Logger.recordOutput("Superstructure/shotData", shotData);
        return shotData.targetFieldYaw().minus(robotState.getEstimatedPose().getRotation());
    }

    private double getTargetFlywheelRPS() {
        ShotData shotData = calculateShotData();
        Logger.recordOutput("Superstructure/shotData", shotData);
        return shotData.flywheelRPS();
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
        InterpolatingMatrixTreeMap<Double, N2, N1> lerpTable = shooter.getLerpTable();
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
