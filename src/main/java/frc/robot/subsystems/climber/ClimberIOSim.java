package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.configs.ClimberConfig;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;

public class ClimberIOSim implements ClimberIO {
    private final DCMotor climberMotorModel = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim climberSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(climberMotorModel, 0.00015, 1.53),
            climberMotorModel
        );

    private ProfiledPIDController climberFeedback;

    private boolean isClimberClosedLoop = true;
    private boolean isClimberEStopped = false;

    private double lastTimeInputs = Timer.getTimestamp();

    private final ClimberConfig config;

    public ClimberIOSim(ClimberConfig config) {
        this.config = config;
        double maxVelRadPerSec = config.climberMaxVelocityRotationsPerSec * (2 * Math.PI);
        double maxAccelRadPerSec2 = config.climberMaxAccelerationRotationsPerSec2 * (2 * Math.PI);
        climberFeedback =
            new ProfiledPIDController(
                config.climberKP,
                config.climberKI,
                config.climberKD,
                new TrapezoidProfile.Constraints(maxVelRadPerSec, maxAccelRadPerSec2)
            );

        climberSim.setState(config.climberStartingPositionRotations * (2 * Math.PI), 0);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        double dt = Timer.getTimestamp() - lastTimeInputs;
        lastTimeInputs = Timer.getTimestamp();

        if (isClimberEStopped) {
            climberSim.setInputVoltage(0);
        } else if (isClimberClosedLoop) {
            climberSim.setInputVoltage(
                MathUtil.clamp(
                    climberFeedback.calculate(climberSim.getAngularPositionRad()),
                    -12,
                    12
                )
            );
        }

        climberSim.update(dt);

        inputs.positionRotations = climberSim.getAngularPositionRotations();
        inputs.velocityRotationsPerSec = climberSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
        inputs.appliedVolts = climberSim.getInputVoltage();
        inputs.torqueCurrent = climberSim.getCurrentDrawAmps();
        inputs.temperatureFahrenheit = 70.0;
    }

    @Override
    public void setPosition(double positionRotations) {
        if (isClimberEStopped) {
            climberSim.setInputVoltage(0);
            isClimberClosedLoop = false;
            return;
        }
        double clampedPosition = MathUtil.clamp(positionRotations,
            config.climberMinPositionRotations,
            config.climberMaxPositionRotations);
        climberFeedback.setGoal(clampedPosition * (2 * Math.PI));
        isClimberClosedLoop = true;
    }

    @Override
    public void setVoltage(double voltage) {
        climberSim.setInputVoltage(isClimberEStopped ? 0 : voltage);
        isClimberClosedLoop = false;
    }

    @Override
    public void configureControlLoop(MotorControlLoopConfig config) {
        climberFeedback.setP(config.kP());
        climberFeedback.setI(config.kI());
        climberFeedback.setD(config.kD());
    }

    @Override
    public void enableEStop() {
        isClimberEStopped = true;
        climberSim.setInputVoltage(0);
        isClimberClosedLoop = false;
    }

    @Override
    public void disableEStop() {
        isClimberEStopped = false;
    }
}
