package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.configs.ShooterConfig;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;

public class ShooterIOSim implements ShooterIO {
    private final DCMotor hoodMotorModel = DCMotor.getKrakenX60Foc(1);
    private final DCMotor turretMotorModel = DCMotor.getKrakenX60Foc(1);
    // 2 motors for flywheel with 1:1 gearing
    private final DCMotor flywheelMotorModel = DCMotor.getKrakenX60Foc(2);

    private final DCMotorSim hoodSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(hoodMotorModel, 0.00015, 21.428), // magic number because hood is not important
            hoodMotorModel
        );
    private final DCMotorSim turretSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turretMotorModel, 0.00015, 21.428),
            turretMotorModel
        );
    private final DCMotorSim flywheelSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(flywheelMotorModel, 0.00015, 1.53),
            flywheelMotorModel
        );

    private PIDController hoodFeedback;
    private ProfiledPIDController turretFeedback;
    private PIDController flywheelFeedback;

    private SimpleMotorFeedforward flywheelFeedforward;

    private boolean isHoodClosedLoop = true;
    private boolean isTurretClosedLoop = true;
    private boolean isFlywheelClosedLoop = true;

    private double desiredFlywheelVelocityRotationsPerSec = 0;

    private double lastTimeInputs = Timer.getTimestamp();

    private final ShooterConfig config;

    public ShooterIOSim(ShooterConfig config) {
        this.config = config;

        // Initialize PID controllers with config values
        hoodFeedback = new PIDController(config.hoodKP, config.hoodKI, config.hoodKD);

        // Turret uses a profiled PID controller in radians with trapezoidal constraints
        double turretMaxVelRadPerSec = Math.toRadians(config.turretMaxVelocityDegPerSec);
        double turretMaxAccelRadPerSec2 = Math.toRadians(config.turretMaxAccelerationDegPerSec2);
        turretFeedback =
            new ProfiledPIDController(
                config.turretKP,
                config.turretKI,
                config.turretKD,
                new TrapezoidProfile.Constraints(turretMaxVelRadPerSec, turretMaxAccelRadPerSec2)
            );

        flywheelFeedback = new PIDController(config.flywheelKP, config.flywheelKI, config.flywheelKD);

        // Initialize feedforward controllers with config values
        flywheelFeedforward = new SimpleMotorFeedforward(config.flywheelKS, config.flywheelKV, config.flywheelKA);

        // Initialize hood position to starting angle (config gives rotations)
        hoodSim.setState(config.hoodStartingAngleRotations * 2 * Math.PI, 0);

        // Initialize turret position to starting angle (config gives degrees)
        double turretStartRad = Math.toRadians(config.turretStartingAngleDeg);
        turretSim.setState(turretStartRad, 0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        double dt = Timer.getTimestamp() - lastTimeInputs;
        lastTimeInputs = Timer.getTimestamp();

        if (isHoodClosedLoop) {
            hoodSim.setInputVoltage(
                MathUtil.clamp(
                    hoodFeedback.calculate(hoodSim.getAngularPositionRad()),
                    -12,
                    12
                )
            );
        }

        if (isTurretClosedLoop) {
            turretSim.setInputVoltage(
                MathUtil.clamp(
                    turretFeedback.calculate(turretSim.getAngularPositionRad()),
                    -12,
                    12
                )
            );
        }

        if (isFlywheelClosedLoop) {
            flywheelSim.setInputVoltage(
                MathUtil.clamp(
                    flywheelFeedforward.calculate(desiredFlywheelVelocityRotationsPerSec) +
                    flywheelFeedback.calculate(flywheelSim.getAngularVelocityRadPerSec()),
                    -12,
                    12
                )
            );
        }

        hoodSim.update(dt);
        turretSim.update(dt);
        flywheelSim.update(dt);

        inputs.hoodAngleRotations = hoodSim.getAngularPositionRotations();
        inputs.hoodVelocityRotationsPerSec = hoodSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
        inputs.hoodAppliedVolts = hoodSim.getInputVoltage();

        inputs.turretAngleRotations = turretSim.getAngularPositionRotations();
        inputs.turretVelocityRotationsPerSec = turretSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
        inputs.turretAppliedVolts = turretSim.getInputVoltage();

        inputs.flywheelVelocityRotationsPerSec = flywheelSim.getAngularVelocityRadPerSec();
        inputs.flywheelAppliedVolts = flywheelSim.getInputVoltage();
        inputs.flywheelTorqueCurrent = flywheelSim.getCurrentDrawAmps();

        // Follower simulated as identical to leader
        inputs.flywheelFollowerAppliedVolts = flywheelSim.getInputVoltage();
        inputs.flywheelFollowerTorqueCurrent = flywheelSim.getCurrentDrawAmps();

        inputs.hoodTorqueCurrent = hoodSim.getCurrentDrawAmps();

        // Simulation doesn't have temperature sensors, use default values
        inputs.hoodTemperatureFahrenheit = 70.0;
        inputs.turretTemperatureFahrenheit = 70.0;
        inputs.flywheelTemperatureFahrenheit = 70.0;
        inputs.flywheelFollowerTemperatureFahrenheit = 70.0;
    }

    @Override
    public void setAngle(double angleRotations) {
        // Clamp angle within software limits
        double clampedAngle = MathUtil.clamp(angleRotations,
            config.hoodMinAngleRotations,
            config.hoodMaxAngleRotations);
        hoodFeedback.setSetpoint(clampedAngle * (2 * Math.PI));
        isHoodClosedLoop = true;
    }

    @Override
    public void setTurretAngle(double angleRotations) {
        // Clamp angle within software limits
        double minRot = config.turretMinAngleDeg / 360.0;
        double maxRot = config.turretMaxAngleDeg / 360.0;
        double clampedAngle = MathUtil.clamp(angleRotations, minRot, maxRot);
        double goalRadians = clampedAngle * (2 * Math.PI);
        turretFeedback.setGoal(goalRadians);
        isTurretClosedLoop = true;
    }

    @Override
    public void setShotVelocity(double velocityRotationsPerSec) {
        flywheelFeedback.setSetpoint(velocityRotationsPerSec);
        desiredFlywheelVelocityRotationsPerSec = velocityRotationsPerSec;
        isFlywheelClosedLoop = true;
    }

    @Override
    public void setHoodTorqueCurrentFOC(double torqueCurrentFOC) {
        hoodSim.setInputVoltage(torqueCurrentFOC);
        isHoodClosedLoop = false;
    }

    @Override
    public void setTurretTorqueCurrentFOC(double torqueCurrentFOC) {
        turretSim.setInputVoltage(torqueCurrentFOC);
        isTurretClosedLoop = false;
    }

    @Override
    public void setFlywheelVoltage(double voltage) {
        flywheelSim.setInputVoltage(voltage);
        isFlywheelClosedLoop = false;
    }

    @Override
    public void configureHoodControlLoop(MotorControlLoopConfig config) {
        hoodFeedback.setP(config.kP());
        hoodFeedback.setI(config.kI());
        hoodFeedback.setD(config.kD());
    }

    @Override
    public void configureTurretControlLoop(MotorControlLoopConfig config) {
        turretFeedback.setP(config.kP());
        turretFeedback.setI(config.kI());
        turretFeedback.setD(config.kD());
    }

    @Override
    public void configureFlywheelControlLoop(MotorControlLoopConfig config) {
        flywheelFeedback.setP(config.kP());
        flywheelFeedback.setI(config.kI());
        flywheelFeedback.setD(config.kD());

        flywheelFeedforward.setKs(config.kS());
        flywheelFeedforward.setKv(config.kV());
        flywheelFeedforward.setKa(config.kA());
    }
}
