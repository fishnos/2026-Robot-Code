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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        HOME(Rotation2d.fromDegrees(45.0)),
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
        OFF(0.0),
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
    private Supplier<Rotation2d> turretAngleSupplier = () -> new Rotation2d(0);
    private Supplier<Double> flywheelRPSSupplier = () -> 0.0;

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    private final ShooterConfig config;
    
    private final DashboardMotorControlLoopConfigurator hoodControlLoopConfigurator;
    private final DashboardMotorControlLoopConfigurator turretControlLoopConfigurator;
    private final DashboardMotorControlLoopConfigurator flywheelControlLoopConfigurator;

    private double hoodSetpointRotations = 0.0;
    private double turretSetpointRotations = 0.0;
    private double flywheelSetpointRPS = 0.0;
    private double turretResolvedSetpointDeg = 0.0;
    private boolean turretUsedUnwindFallback = false;

    private Shooter() {
        boolean useSimulation = Constants.shouldUseSimulation(Constants.SimOnlySubsystems.SHOOTER);
        config = ConfigLoader.load(
            "shooter",
            ConfigLoader.getModeFolder(Constants.SimOnlySubsystems.SHOOTER),
            ShooterConfig.class
        );
        shooterIO = useSimulation ? new ShooterIOSim(config) : new ShooterIOTalonFX(config);

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
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);

        // Handle control loop configuration updates
        if (hoodControlLoopConfigurator.hasChanged()) {
            shooterIO.configureHoodControlLoop(hoodControlLoopConfigurator.getConfig());
        }
        if (turretControlLoopConfigurator.hasChanged()) {
            shooterIO.configureTurretControlLoop(turretControlLoopConfigurator.getConfig());
        }
        if (flywheelControlLoopConfigurator.hasChanged()) {
            shooterIO.configureFlywheelControlLoop(flywheelControlLoopConfigurator.getConfig());
        }

    }

    // Supplier setters (called by Superstructure)
    public void setHoodAngleSupplier(Supplier<Rotation2d> supplier) {
        this.hoodAngleSupplier = supplier;
    }

    public void setTurretAngleSupplier(Supplier<Rotation2d> supplier) {
        this.turretAngleSupplier = supplier;
    }

    public void setFlywheelRPSSupplier(Supplier<Double> supplier) {
        this.flywheelRPSSupplier = supplier;
    }

    public void setHoodSetpoint(HoodSetpoint setpoint) {
        Rotation2d target = setpoint == HoodSetpoint.DYNAMIC ? hoodAngleSupplier.get() : setpoint.getAngle();
        setHoodAngle(target);
    }

    public void setTurretSetpoint(TurretSetpoint setpoint) {
        Rotation2d target = setpoint == TurretSetpoint.DYNAMIC ? turretAngleSupplier.get() : setpoint.getAngle();
        setTurretAngle(target);
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
        Logger.recordOutput("Shooter/rawSetpointRotations", clampedAngleRotations);

        shooterIO.setAngle(clampedAngleRotations);
    }

    private void setTurretAngle(Rotation2d angle) {
        double requestedDeg = angle.getDegrees();
        double currentDeg = shooterInputs.turretAngleRotations * 360.0;
        double minDeg = config.turretMinAngleDeg;
        double maxDeg = config.turretMaxAngleDeg;

        Logger.recordOutput("Shooter/unclampedTurretAngleRotations", angle.getRotations());
        Logger.recordOutput("Shooter/turretRequestedAngleDeg", requestedDeg);
        Logger.recordOutput("Shooter/turretCurrentAngleDeg", currentDeg);

        TurretResolution resolution = resolveTurretTargetDegrees(requestedDeg, currentDeg, minDeg, maxDeg);
        double targetAngleDeg = resolution.targetAngleDeg();
        boolean usedUnwindFallback = resolution.usedUnwindFallback();
        double unwindTargetDeg = resolution.unwindTargetDeg();

        double targetAngleRotations = targetAngleDeg / 360.0;
        turretSetpointRotations = targetAngleRotations;
        turretResolvedSetpointDeg = targetAngleDeg;
        turretUsedUnwindFallback = usedUnwindFallback;

        Logger.recordOutput("Shooter/turretUsedUnwindFallback", usedUnwindFallback);
        Logger.recordOutput("Shooter/turretUnwindTargetDeg", unwindTargetDeg);
        Logger.recordOutput("Shooter/turretResolvedSetpointDeg", targetAngleDeg);
        Logger.recordOutput("Shooter/turretAngleSetpointRotations", targetAngleRotations);

        shooterIO.setTurretAngle(targetAngleRotations);
    }

    static TurretResolution resolveTurretTargetDegrees(
        double requestedDeg,
        double currentDeg,
        double minDeg,
        double maxDeg
    ) {
        double targetAngleDeg;
        boolean usedUnwindFallback;
        double unwindTargetDeg = Double.NaN;

        int minK = (int) Math.ceil((minDeg - requestedDeg) / 360.0);
        int maxK = (int) Math.floor((maxDeg - requestedDeg) / 360.0);
        int currentBranchK = (int) Math.round((currentDeg - requestedDeg) / 360.0);
        double currentBranchTargetDeg = requestedDeg + currentBranchK * 360.0;
        double currentBranchClampedDeg = MathUtil.clamp(currentBranchTargetDeg, minDeg, maxDeg);
        boolean isCurrentBranchOutsideLimits = currentBranchClampedDeg != currentBranchTargetDeg;

        if (minK <= maxK) {
            double bestAngleDeg = requestedDeg + (minK * 360.0);
            double bestDistanceDeg = Math.abs(bestAngleDeg - currentDeg);

            for (int k = minK + 1; k <= maxK; k++) {
                double candidateDeg = requestedDeg + (k * 360.0);
                double candidateDistanceDeg = Math.abs(candidateDeg - currentDeg);
                if (candidateDistanceDeg < bestDistanceDeg) {
                    bestDistanceDeg = candidateDistanceDeg;
                    bestAngleDeg = candidateDeg;
                }
            }

            if (isCurrentBranchOutsideLimits) {
                double clampedCurrentBranchDistanceDeg = Math.abs(currentBranchClampedDeg - currentDeg);
                if (clampedCurrentBranchDistanceDeg < bestDistanceDeg) {
                    bestAngleDeg = currentBranchClampedDeg;
                }
            }

            targetAngleDeg = bestAngleDeg;
            usedUnwindFallback = false;
        } else {
            unwindTargetDeg = Math.abs(minDeg) <= Math.abs(maxDeg) ? minDeg : maxDeg;
            targetAngleDeg = unwindTargetDeg;
            usedUnwindFallback = true;
        }

        return new TurretResolution(targetAngleDeg, usedUnwindFallback, unwindTargetDeg);
    }

    static record TurretResolution(
        double targetAngleDeg,
        boolean usedUnwindFallback,
        double unwindTargetDeg
    ) {}

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
        return Math.abs(shooterInputs.hoodAngleRotations - hoodSetpointRotations)
            < (config.hoodAngleToleranceDegrees / 360.0);
    }

    @AutoLogOutput(key = "Shooter/isTurretAtSetpoint")
    public boolean isTurretAtSetpoint() {
        return Math.abs(shooterInputs.turretAngleRotations - turretSetpointRotations) < config.turretAngleToleranceRotations;
    }

    public double getTurretSetpointDegrees() {
        return turretResolvedSetpointDeg;
    }

    public boolean getTurretUsedUnwindFallback() {
        return turretUsedUnwindFallback;
    }

    @AutoLogOutput(key = "Shooter/isFlywheelAtSetpoint")
    public boolean isFlywheelAtSetpoint() {
        return Math.abs(shooterInputs.flywheelVelocityRotationsPerSec - flywheelSetpointRPS) < config.flywheelVelocityToleranceRPS;
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
