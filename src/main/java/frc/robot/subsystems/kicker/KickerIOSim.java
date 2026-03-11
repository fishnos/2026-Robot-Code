package frc.robot.subsystems.kicker;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.configs.KickerConfig;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;

public class KickerIOSim implements KickerIO {
    private final DCMotor kickerMotorModel = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim kickerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(kickerMotorModel, 0.00015, 1.53),
            kickerMotorModel
        );

    private PIDController kickerFeedback;
    private SimpleMotorFeedforward kickerFeedforward;

    private boolean isKickerClosedLoop = true;
    private double desiredKickerVelocityRotationsPerSec = 0;
    private boolean isKickerEStopped = false;

    private double lastTimeInputs = Timer.getTimestamp();

    private final KickerConfig config;

    public KickerIOSim(KickerConfig config) {
        this.config = config;

        // Initialize PID controller with config values
        kickerFeedback = new PIDController(config.kickerKP, config.kickerKI, config.kickerKD);

        // Initialize feedforward controller with config values
        kickerFeedforward = new SimpleMotorFeedforward(config.kickerKS, config.kickerKV, config.kickerKA);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        double dt = Timer.getTimestamp() - lastTimeInputs;
        lastTimeInputs = Timer.getTimestamp();

        if (isKickerEStopped) {
            kickerSim.setInputVoltage(0);
        } else if (isKickerClosedLoop) {
            kickerSim.setInputVoltage(
                MathUtil.clamp(
                    kickerFeedforward.calculate(desiredKickerVelocityRotationsPerSec) +
                    kickerFeedback.calculate(kickerSim.getAngularVelocityRadPerSec() / (2 * Math.PI)),
                    -12,
                    12
                )
            );
        }

        kickerSim.update(dt);

        inputs.velocityRotationsPerSec = kickerSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
        inputs.appliedVolts = kickerSim.getInputVoltage();
        inputs.torqueCurrent = kickerSim.getCurrentDrawAmps();

        // Simulation doesn't have temperature sensors, use default values
        inputs.temperatureFahrenheit = 70.0;
    }

    @Override
    public void setVelocity(double velocityRotationsPerSec) {
        if (isKickerEStopped) {
            kickerSim.setInputVoltage(0);
            isKickerClosedLoop = false;
            return;
        }
        kickerFeedback.setSetpoint(velocityRotationsPerSec);
        desiredKickerVelocityRotationsPerSec = velocityRotationsPerSec;
        isKickerClosedLoop = true;
    }

    @Override
    public void setVoltage(double voltage) {
        kickerSim.setInputVoltage(isKickerEStopped ? 0 : voltage);
        isKickerClosedLoop = false;
    }

    @Override
    public void configureControlLoop(MotorControlLoopConfig config) {
        kickerFeedback.setP(config.kP());
        kickerFeedback.setI(config.kI());
        kickerFeedback.setD(config.kD());

        kickerFeedforward.setKs(config.kS());
        kickerFeedforward.setKv(config.kV());
        kickerFeedforward.setKa(config.kA());
    }

    @Override
    public void enableEStop() {
        isKickerEStopped = true;
        kickerSim.setInputVoltage(0);
        isKickerClosedLoop = false;
    }

    @Override
    public void disableEStop() {
        isKickerEStopped = false;
    }
}
