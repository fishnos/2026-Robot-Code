package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.configs.ShooterConfig;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.util.ConfigLoader;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator;

public class Shooter extends SubsystemBase {
    private static Shooter instance = null;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    public enum HoodSetpoint {
        HOME(Rotation2d.fromDegrees(70.0)),
        DYNAMIC(null);

        private final Rotation2d angle;
        
        HoodSetpoint(Rotation2d angle) {
            this.angle = angle;
        }

        public Rotation2d getAngle() {
            return angle;
        }
    }

    public enum TurretSetpoint {
        HOME(Rotation2d.fromDegrees(180.0)),
        DYNAMIC(null);

        private final Rotation2d angle;
        TurretSetpoint(Rotation2d angle) {
            this.angle = angle;
        }

        public Rotation2d getAngle() {
            return angle;
        }
    }

    public enum FlywheelSetpoint {
        OFF(-1.0), // prevent bang bang jitter
        DYNAMIC(Double.NaN);

        private final double rps;
        FlywheelSetpoint(double rps) {
            this.rps = rps;
        }

        public double getRps() {
            return rps;
        }
    }

    // Setpoint Suppliers (set by Superstructure)
    private Supplier<Rotation2d> hoodAngleSupplier = () -> Rotation2d.fromDegrees(45.0);
    // Dynamic turret tracking uses field-relative targets. Direct turret setters remain robot-relative.
    private Supplier<Rotation2d> turretAngleSupplier = () -> new Rotation2d(0);
    private Supplier<Double> turretVelocitySupplier = () -> 0.0;
    private Supplier<Double> flywheelRPSSupplier = () -> 0.0;

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    private final ShooterConfig config;
    
    private final DashboardMotorControlLoopConfigurator hoodControlLoopConfigurator;
    private final DashboardMotorControlLoopConfigurator turretControlLoopConfigurator;
    private final DashboardMotorControlLoopConfigurator flywheelControlLoopConfigurator;
    private boolean pendingHoodControlLoopConfigApply = false;
    private boolean pendingTurretControlLoopConfigApply = false;
    private boolean pendingFlywheelControlLoopConfigApply = false;

    private double hoodSetpointRotations = 0.0;
    private double turretSetpointRotations = 0.0;
    private double flywheelSetpointRPS = 0.0;
    private double turretResolvedSetpointDeg = 0.0;
    private boolean turretUsedUnwindFallback = false;
    private final double turretMaxVelocityRotPerSec;
    private final double turretMaxAccelerationRotPerSec2;
    private State turretProfileSetpoint;
    private double lastTurretGoalDeg;
    private final boolean enableConnectionAlerts;
    private final Alert hoodDisconnectedAlert;
    private final Alert turretDisconnectedAlert;
    private final Alert flywheelDisconnectedAlert;
    private final Alert flywheelFollowerDisconnectedAlert;

    private Shooter() {
        boolean useSimulation = Constants.shouldUseSimulation(Constants.SimOnlySubsystems.SHOOTER);
        config = ConfigLoader.load(
            "shooter",
            ConfigLoader.getModeFolder(Constants.SimOnlySubsystems.SHOOTER),
            ShooterConfig.class
        );
        shooterIO = useSimulation ? new ShooterIOSim(config) : new ShooterIOTalonFX(config);
        enableConnectionAlerts = !useSimulation && Constants.currentMode != Constants.Mode.REPLAY;
        hoodDisconnectedAlert = new Alert("Shooter hood motor is disconnected.", AlertType.kWarning);
        turretDisconnectedAlert = new Alert("Shooter turret motor is disconnected.", AlertType.kWarning);
        flywheelDisconnectedAlert = new Alert("Shooter flywheel motor is disconnected.", AlertType.kWarning);
        flywheelFollowerDisconnectedAlert =
            new Alert("Shooter flywheel follower motor is disconnected.", AlertType.kWarning);

        hoodControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Shooter/hoodControlLoop", 
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.hoodKP,
                config.hoodKI,
                config.hoodKD,
                config.hoodKS,
                config.hoodKV,
                config.hoodKA
            )
        );
        turretControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Shooter/turretControlLoop", 
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.turretKP,
                config.turretKI,
                config.turretKD,
                config.turretKS,
                config.turretKV,
                config.turretKA
            )
        );
        flywheelControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Shooter/flywheelControlLoop", 
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.flywheelKP,
                config.flywheelKI,
                config.flywheelKD,
                config.flywheelKS,
                config.flywheelKV,
                config.flywheelKA
            )
        );

        turretMaxVelocityRotPerSec = config.turretMaxVelocityDegPerSec / 360.0;
        turretMaxAccelerationRotPerSec2 = config.turretMaxAccelerationDegPerSec2 / 360.0;
        turretProfileSetpoint = new State(config.turretStartingAngleDeg / 360.0, 0.0);
        lastTurretGoalDeg = config.turretStartingAngleDeg;
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);

        hoodDisconnectedAlert.set(enableConnectionAlerts && !shooterInputs.hoodMotorConnected);
        turretDisconnectedAlert.set(enableConnectionAlerts && !shooterInputs.turretMotorConnected);
        flywheelDisconnectedAlert.set(enableConnectionAlerts && !shooterInputs.flywheelMotorConnected);
        flywheelFollowerDisconnectedAlert.set(enableConnectionAlerts && !shooterInputs.flywheelFollowerMotorConnected);

        // Handle control loop configuration updates
        pendingHoodControlLoopConfigApply |= hoodControlLoopConfigurator.hasChanged();
        pendingTurretControlLoopConfigApply |= turretControlLoopConfigurator.hasChanged();
        pendingFlywheelControlLoopConfigApply |= flywheelControlLoopConfigurator.hasChanged();
        if (DriverStation.isDisabled()) {
            if (pendingHoodControlLoopConfigApply) {
                shooterIO.configureHoodControlLoop(hoodControlLoopConfigurator.getConfig());
                pendingHoodControlLoopConfigApply = false;
            }
            if (pendingTurretControlLoopConfigApply) {
                shooterIO.configureTurretControlLoop(turretControlLoopConfigurator.getConfig());
                pendingTurretControlLoopConfigApply = false;
            }
            if (pendingFlywheelControlLoopConfigApply) {
                shooterIO.configureFlywheelControlLoop(flywheelControlLoopConfigurator.getConfig());
                pendingFlywheelControlLoopConfigApply = false;
            }

            double clampedMeasuredTurretRotations = MathUtil.clamp(
                shooterInputs.turretAngleRotations,
                config.turretMinAngleDeg / 360.0,
                config.turretMaxAngleDeg / 360.0
            );
            turretProfileSetpoint = new State(clampedMeasuredTurretRotations, 0.0);
            lastTurretGoalDeg = clampedMeasuredTurretRotations * 360.0;
        }
    }

    // Supplier setters (called by Superstructure)
    public void setHoodAngleSupplier(Supplier<Rotation2d> supplier) {
        this.hoodAngleSupplier = supplier;
    }

    public void setTurretAngleSupplier(Supplier<Rotation2d> supplier) {
        this.turretAngleSupplier = supplier;
    }

    public void setTurretVelocitySupplier(Supplier<Double> supplier) {
        this.turretVelocitySupplier = supplier;
    }

    public void setFlywheelRPSSupplier(Supplier<Double> supplier) {
        this.flywheelRPSSupplier = supplier;
    }

    public void setHoodSetpoint(HoodSetpoint setpoint) {
        Rotation2d target = setpoint == HoodSetpoint.DYNAMIC ? hoodAngleSupplier.get() : setpoint.getAngle();
        setHoodAngle(target);
    }

    public void setTurretSetpoint(TurretSetpoint setpoint) {
        if (setpoint == TurretSetpoint.DYNAMIC) {
            setTurretAngle(
                turretAngleSupplier.get(),
                turretVelocitySupplier.get(),
                TurretReferenceFrame.FIELD_RELATIVE
            );
        } else {
            setTurretAngle(setpoint.getAngle(), 0.0, TurretReferenceFrame.ROBOT_RELATIVE);
        }
    }

    public void setFlywheelSetpoint(FlywheelSetpoint setpoint) {
        double target = setpoint == FlywheelSetpoint.DYNAMIC ? flywheelRPSSupplier.get() : setpoint.getRps();
        setShotVelocity(target);
    }

    // Direct setters for mechanism control (exposed for tuning/overrides in RobotContainer)
    public void setHoodAngle(Rotation2d angle) {
        double clampedAngleDeg = MathUtil.clamp(
            angle.getDegrees(),
            config.hoodMinAngleDegrees,
            config.hoodMaxAngleDegrees
        );
        double clampedAngleRotations = clampedAngleDeg / 360.0;

        hoodSetpointRotations = clampedAngleRotations;
        Logger.recordOutput("Shooter/hoodSetpointDegrees", clampedAngleDeg);
        Logger.recordOutput("Shooter/angleSetpointRotations", clampedAngleRotations);
        Logger.recordOutput("Shooter/rawSetpointRotations", angle.getRotations());

        shooterIO.setAngle(clampedAngleRotations);
    }

    public void setTurretAngle(Rotation2d angle) {
        setTurretAngle(angle, 0.0, TurretReferenceFrame.ROBOT_RELATIVE);
    }

    public void setTurretAngle(Rotation2d angle, double requestedVelocityRotPerSec) {
        setTurretAngle(angle, requestedVelocityRotPerSec, TurretReferenceFrame.ROBOT_RELATIVE);
    }

    private void setTurretAngle(
        Rotation2d angle,
        double requestedVelocityRotPerSec,
        TurretReferenceFrame referenceFrame
    ) {
        Rotation2d robotYaw = RobotState.getInstance().getEstimatedPose().getRotation();
        double robotYawVelocityRotPerSec = RobotState.getInstance().getYawVelocityRadPerSec() / (2.0 * Math.PI);
        double minDeg = config.turretMinAngleDeg;
        double maxDeg = config.turretMaxAngleDeg;

        TurretGoalState goalState = resolveTurretGoalState(
            angle,
            requestedVelocityRotPerSec,
            referenceFrame,
            robotYaw,
            robotYawVelocityRotPerSec,
            lastTurretGoalDeg,
            minDeg,
            maxDeg
        );
        State mechanismGoalState = new State(
            goalState.targetAngleDeg() / 360.0,
            goalState.targetVelocityRotPerSec()
        );
        turretProfileSetpoint = calculateTurretProfileSetpoint(
            turretProfileSetpoint,
            mechanismGoalState,
            turretMaxVelocityRotPerSec,
            turretMaxAccelerationRotPerSec2,
            Constants.kLOOP_CYCLE_MS
        );
        lastTurretGoalDeg = goalState.targetAngleDeg();

        turretSetpointRotations = mechanismGoalState.position;
        turretResolvedSetpointDeg = goalState.targetAngleDeg();
        turretUsedUnwindFallback = goalState.usedUnwindFallback();

        Logger.recordOutput("Shooter/unclampedTurretAngleRotations", angle.getRotations());
        Logger.recordOutput("Shooter/turretFieldRelativeRequestedAngleDeg", goalState.fieldRelativeAngleDeg());
        Logger.recordOutput(
            "Shooter/turretFieldRelativeRequestedVelocityRotationsPerSec",
            goalState.fieldRelativeVelocityRotPerSec()
        );
        Logger.recordOutput("Shooter/turretRobotRelativeRequestedAngleDeg", goalState.robotRelativeAngleDeg());
        Logger.recordOutput(
            "Shooter/turretRobotRelativeRequestedVelocityRotationsPerSec",
            goalState.robotRelativeVelocityRotPerSec()
        );
        Logger.recordOutput("Shooter/turretUsedUnwindFallback", goalState.usedUnwindFallback());
        Logger.recordOutput("Shooter/turretUnwindTargetDeg", goalState.unwindTargetDeg());
        Logger.recordOutput("Shooter/turretResolvedSetpointDeg", goalState.targetAngleDeg());
        Logger.recordOutput("Shooter/turretAngleSetpointRotations", mechanismGoalState.position);
        Logger.recordOutput("Shooter/turretRequestedVelocityRotationsPerSec", goalState.targetVelocityRotPerSec());
        Logger.recordOutput("Shooter/turretProfileSetpointRotations", turretProfileSetpoint.position);
        Logger.recordOutput(
            "Shooter/turretProfileSetpointVelocityRotationsPerSec",
            turretProfileSetpoint.velocity
        );

        shooterIO.setTurretAngle(turretProfileSetpoint.position, turretProfileSetpoint.velocity);
    }

    static TurretGoalState resolveTurretGoalState(
        Rotation2d requestedAngle,
        double requestedVelocityRotPerSec,
        TurretReferenceFrame referenceFrame,
        Rotation2d robotYaw,
        double robotYawVelocityRotPerSec,
        double previousGoalDeg,
        double minDeg,
        double maxDeg
    ) {
        double normalizedRequestedVelocityRotPerSec =
            Double.isFinite(requestedVelocityRotPerSec) ? requestedVelocityRotPerSec : 0.0;
        Rotation2d robotRelativeAngle = requestedAngle;
        double robotRelativeVelocityRotPerSec = normalizedRequestedVelocityRotPerSec;
        double fieldRelativeAngleDeg = Double.NaN;
        double fieldRelativeVelocityRotPerSec = Double.NaN;

        if (referenceFrame == TurretReferenceFrame.FIELD_RELATIVE) {
            fieldRelativeAngleDeg = requestedAngle.getDegrees();
            fieldRelativeVelocityRotPerSec = normalizedRequestedVelocityRotPerSec;
            robotRelativeAngle = requestedAngle.minus(robotYaw);
            robotRelativeVelocityRotPerSec = normalizedRequestedVelocityRotPerSec - robotYawVelocityRotPerSec;
        }

        TurretResolution resolution = resolveTurretTargetDegrees(
            robotRelativeAngle.getDegrees(),
            previousGoalDeg,
            minDeg,
            maxDeg
        );
        double targetVelocityRotPerSec = resolution.usedUnwindFallback() ? 0.0 : robotRelativeVelocityRotPerSec;

        return new TurretGoalState(
            fieldRelativeAngleDeg,
            fieldRelativeVelocityRotPerSec,
            robotRelativeAngle.getDegrees(),
            robotRelativeVelocityRotPerSec,
            resolution.targetAngleDeg(),
            targetVelocityRotPerSec,
            resolution.usedUnwindFallback(),
            resolution.unwindTargetDeg()
        );
    }

    static State calculateTurretProfileSetpoint(
        State currentSetpoint,
        State goalState,
        double maxVelocityRotPerSec,
        double maxAccelerationRotPerSec2,
        double loopPeriodSec
    ) {
        return new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocityRotPerSec, maxAccelerationRotPerSec2)
        ).calculate(loopPeriodSec, currentSetpoint, goalState);
    }

    static TurretResolution resolveTurretTargetDegrees(
        double requestedDeg,
        double referenceDeg,
        double minDeg,
        double maxDeg
    ) {
        double targetAngleDeg;
        boolean usedUnwindFallback;
        double unwindTargetDeg = Double.NaN;

        int minK = (int) Math.ceil((minDeg - requestedDeg) / 360.0);
        int maxK = (int) Math.floor((maxDeg - requestedDeg) / 360.0);
        int currentBranchK = (int) Math.round((referenceDeg - requestedDeg) / 360.0);
        double currentBranchTargetDeg = requestedDeg + currentBranchK * 360.0;
        double currentBranchClampedDeg = MathUtil.clamp(currentBranchTargetDeg, minDeg, maxDeg);
        boolean isCurrentBranchOutsideLimits = currentBranchClampedDeg != currentBranchTargetDeg;

        if (minK <= maxK) {
            double bestAngleDeg = requestedDeg + (minK * 360.0);
            double bestDistanceDeg = Math.abs(bestAngleDeg - referenceDeg);

            for (int k = minK + 1; k <= maxK; k++) {
                double candidateDeg = requestedDeg + (k * 360.0);
                double candidateDistanceDeg = Math.abs(candidateDeg - referenceDeg);
                if (candidateDistanceDeg < bestDistanceDeg) {
                    bestDistanceDeg = candidateDistanceDeg;
                    bestAngleDeg = candidateDeg;
                }
            }

            if (isCurrentBranchOutsideLimits) {
                double clampedCurrentBranchDistanceDeg = Math.abs(currentBranchClampedDeg - referenceDeg);
                double clampedCurrentBranchAimErrorDeg = Math.abs(
                    MathUtil.inputModulus(requestedDeg - currentBranchClampedDeg, -180.0, 180.0)
                );
                // Keep the turret pinned at the soft limit only when that bound still roughly tracks the request.
                if (clampedCurrentBranchDistanceDeg < bestDistanceDeg && clampedCurrentBranchAimErrorDeg <= 90.0) {
                    bestAngleDeg = currentBranchClampedDeg;
                }
            }

            targetAngleDeg = bestAngleDeg;
            usedUnwindFallback = false;
        } else {
            unwindTargetDeg = selectFallbackTurretBoundDegrees(
                requestedDeg,
                referenceDeg,
                minDeg,
                maxDeg,
                currentBranchClampedDeg
            );
            targetAngleDeg = unwindTargetDeg;
            usedUnwindFallback = true;
        }

        return new TurretResolution(targetAngleDeg, usedUnwindFallback, unwindTargetDeg);
    }

    static double selectFallbackTurretBoundDegrees(
        double requestedDeg,
        double referenceDeg,
        double minDeg,
        double maxDeg,
        double currentBranchClampedDeg
    ) {
        double minAimErrorDeg = Math.abs(MathUtil.inputModulus(requestedDeg - minDeg, -180.0, 180.0));
        double maxAimErrorDeg = Math.abs(MathUtil.inputModulus(requestedDeg - maxDeg, -180.0, 180.0));
        if (minAimErrorDeg < maxAimErrorDeg) {
            return minDeg;
        }
        if (maxAimErrorDeg < minAimErrorDeg) {
            return maxDeg;
        }

        double minCurrentDistanceDeg = Math.abs(minDeg - referenceDeg);
        double maxCurrentDistanceDeg = Math.abs(maxDeg - referenceDeg);
        if (minCurrentDistanceDeg < maxCurrentDistanceDeg) {
            return minDeg;
        }
        if (maxCurrentDistanceDeg < minCurrentDistanceDeg) {
            return maxDeg;
        }

        return currentBranchClampedDeg;
    }

    static record TurretResolution(
        double targetAngleDeg,
        boolean usedUnwindFallback,
        double unwindTargetDeg
    ) {}

    static record TurretGoalState(
        double fieldRelativeAngleDeg,
        double fieldRelativeVelocityRotPerSec,
        double robotRelativeAngleDeg,
        double robotRelativeVelocityRotPerSec,
        double targetAngleDeg,
        double targetVelocityRotPerSec,
        boolean usedUnwindFallback,
        double unwindTargetDeg
    ) {}

    enum TurretReferenceFrame {
        ROBOT_RELATIVE,
        FIELD_RELATIVE
    }

    public void setShotVelocity(double velocityRotationsPerSec) {
        flywheelSetpointRPS = velocityRotationsPerSec;
        Logger.recordOutput("Shooter/shotVelocitySetpointRotationsPerSec", velocityRotationsPerSec);
        shooterIO.setShotVelocity(velocityRotationsPerSec);
    }

    // Mechanism getters
    public double getHoodAngleRotations() {
        return shooterInputs.hoodAngleRotations;
    }

    public double getTurretAngleRotations() {
        return shooterInputs.turretAngleRotations;
    }

    public double getFlywheelVelocityRotationsPerSec() {
        return shooterInputs.flywheelVelocityRotationsPerSec;
    }

    public double getHoodSetpointRotations() {
        return hoodSetpointRotations;
    }

    public double getTurretSetpointRotations() {
        return turretSetpointRotations;
    }

    public double getFlywheelSetpointRPS() {
        return flywheelSetpointRPS;
    }

    public void enableHoodEStop() {
        shooterIO.enableHoodEStop();
    }

    public void disableHoodEStop() {
        shooterIO.disableHoodEStop();
    }

    public void enableTurretEStop() {
        shooterIO.enableTurretEStop();
    }

    public void disableTurretEStop() {
        shooterIO.disableTurretEStop();
    }

    public void enableFlywheelEStop() {
        shooterIO.enableFlywheelEStop();
    }

    public void disableFlywheelEStop() {
        shooterIO.disableFlywheelEStop();
    }

    /**
     * Calculate ball backspin from differential roller surface speeds.
     * Flywheel (bottom) moving faster than back roller (top) creates backspin.
     */
    public double calculateBackSpinRPM(double flywheelVelocityRotationsPerSec) {
        return calculateBackSpinRPM(
            flywheelVelocityRotationsPerSec,
            config.flywheelRadiusMeters,
            config.backRollerGearRatio,
            config.backRollerRadiusMeters,
            FieldConstants.fuelDiameter / 2.0
        );
    }

    public double calculateShotExitVelocityMetersPerSec(double flywheelVelocityRotationsPerSec) {
        return calculateShotExitVelocityMetersPerSec(
            flywheelVelocityRotationsPerSec,
            config.flywheelRadiusMeters,
            config.backRollerGearRatio,
            config.backRollerRadiusMeters
        );
    }

    static double calculateBackSpinRPM(
        double flywheelVelocityRotationsPerSec,
        double flywheelRadiusMeters,
        double backRollerGearRatio,
        double backRollerRadiusMeters,
        double ballRadiusMeters
    ) {
        double flywheelSurfaceVel = flywheelVelocityRotationsPerSec * 2 * Math.PI * flywheelRadiusMeters;
        double backRollerSurfaceVel = flywheelVelocityRotationsPerSec * backRollerGearRatio
            * 2 * Math.PI * backRollerRadiusMeters;

        double deltaV = flywheelSurfaceVel - backRollerSurfaceVel;
        double spinRadPerSec = deltaV / ballRadiusMeters;

        return spinRadPerSec * 60.0 / (2 * Math.PI);
    }

    static double calculateShotExitVelocityMetersPerSec(
        double flywheelVelocityRotationsPerSec,
        double flywheelRadiusMeters,
        double backRollerGearRatio,
        double backRollerRadiusMeters
    ) {
        double flywheelSurfaceVel = flywheelVelocityRotationsPerSec * 2 * Math.PI * flywheelRadiusMeters;
        double backRollerSurfaceVel = flywheelVelocityRotationsPerSec * backRollerGearRatio
            * 2 * Math.PI * backRollerRadiusMeters;
        return (flywheelSurfaceVel + backRollerSurfaceVel) / 2.0;
    }

    public double getShotExitVelocityMetersPerSec() {
        return calculateShotExitVelocityMetersPerSec(shooterInputs.flywheelVelocityRotationsPerSec);
    }

    // Setpoint check methods
    @AutoLogOutput(key = "Shooter/isHoodAtSetpoint")
    public boolean isHoodAtSetpoint() {
        return shooterInputs.hoodMotorConnected
            && Math.abs(shooterInputs.hoodAngleRotations - hoodSetpointRotations)
            < (config.hoodAngleToleranceDegrees / 360.0);
    }

    @AutoLogOutput(key = "Shooter/isTurretAtSetpoint")
    public boolean isTurretAtSetpoint() {
        return shooterInputs.turretMotorConnected
            && Math.abs(shooterInputs.turretAngleRotations - turretSetpointRotations) < config.turretAngleToleranceRotations;
    }

    public double getTurretSetpointDegrees() {
        return turretResolvedSetpointDeg;
    }

    public double getTurretAngleToleranceDegrees() {
        return config.turretAngleToleranceRotations * 360.0;
    }

    public boolean getTurretUsedUnwindFallback() {
        return turretUsedUnwindFallback;
    }

    @AutoLogOutput(key = "Shooter/isFlywheelAtSetpoint")
    public boolean isFlywheelAtSetpoint() {
        return shooterInputs.flywheelMotorConnected
            && Math.abs(shooterInputs.flywheelVelocityRotationsPerSec - flywheelSetpointRPS) < config.flywheelVelocityToleranceRPS;
    }

    @AutoLogOutput(key = "Shooter/isHoodMotorConnected")
    public boolean isHoodMotorConnected() {
        return shooterInputs.hoodMotorConnected;
    }

    @AutoLogOutput(key = "Shooter/isTurretMotorConnected")
    public boolean isTurretMotorConnected() {
        return shooterInputs.turretMotorConnected;
    }

    @AutoLogOutput(key = "Shooter/isFlywheelMotorConnected")
    public boolean isFlywheelMotorConnected() {
        return shooterInputs.flywheelMotorConnected;
    }

    @AutoLogOutput(key = "Shooter/isFlywheelFollowerMotorConnected")
    public boolean isFlywheelFollowerMotorConnected() {
        return shooterInputs.flywheelFollowerMotorConnected;
    }

    // Config getters
    public InterpolatingMatrixTreeMap<Double, N3, N1> getLerpTable() {
        return config.getLerpTable();
    }

    public InterpolatingMatrixTreeMap<Double, N3, N1> getPassLerpTable() {
        return config.getPassLerpTable();
    }

    public Pose3d getShooterRelativePose() {
        return config.getShooterPose3d();
    }

    public double getMinShotDistFromShooterMeters() {
        return config.minShotDistFromShooterMeters;
    }

    public double getMaxShotDistFromShooterMeters() {
        return config.maxShotDistFromShooterMeters;
    }

    public double getMinPassDistFromShooterMeters() {
        return config.minPassDistFromShooterMeters;
    }

    public double getMaxPassDistFromShooterMeters() {
        return config.maxPassDistFromShooterMeters;
    }

    public double getLatencyCompensationSeconds() {
        return config.latencyCompensationSeconds;
    }

    public double getTurretMinAngleDeg() {
        return config.turretMinAngleDeg;
    }

    public double getTurretMaxAngleDeg() {
        return config.turretMaxAngleDeg;
    }

    public double getTurretAngleToleranceDeg() {
        return config.turretAngleToleranceRotations * 360.0;
    }
}
