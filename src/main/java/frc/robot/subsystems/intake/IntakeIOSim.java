package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.configs.IntakeConfig;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;

public class IntakeIOSim implements IntakeIO {
    private final DCMotor rollerMotorModel = DCMotor.getKrakenX60Foc(1);
    private final DCMotor pivotMotorModel = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(rollerMotorModel, 0.00015, 1.53),
            rollerMotorModel
        );

    private final DCMotorSim pivotSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(pivotMotorModel, 0.00015, 21.428),
            pivotMotorModel
        );

    private PIDController rollerFeedback;
    private SimpleMotorFeedforward rollerFeedforward;
    private PIDController pivotFeedback;

    private boolean isRollerClosedLoop = true;
    private boolean isPivotClosedLoop = true;
    private double desiredRollerVelocityRotationsPerSec = 0;

    private double lastTimeInputs = Timer.getTimestamp();
    private final IntakeConfig config;

    public IntakeIOSim(IntakeConfig config) {
        this.config = config;
        rollerFeedback = new PIDController(config.rollerKP, config.rollerKI, config.rollerKD);
        rollerFeedforward = new SimpleMotorFeedforward(config.rollerKS, config.rollerKV, config.rollerKA);
        pivotFeedback = new PIDController(config.pivotKP, config.pivotKI, config.pivotKD);

        pivotSim.setState(config.pivotStartingAngleRotations * 2 * Math.PI, 0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double dt = Timer.getTimestamp() - lastTimeInputs;
        lastTimeInputs = Timer.getTimestamp();

        if (isRollerClosedLoop) {
            rollerSim.setInputVoltage(
                MathUtil.clamp(
                    rollerFeedforward.calculate(desiredRollerVelocityRotationsPerSec) +
                    rollerFeedback.calculate(rollerSim.getAngularVelocityRadPerSec()),
                    -12,
                    12
                )
            );
        }

        if (isPivotClosedLoop) {
            pivotSim.setInputVoltage(
                MathUtil.clamp(
                    pivotFeedback.calculate(pivotSim.getAngularPositionRad()),
                    -12,
                    12
                )
            );
        }

        rollerSim.update(dt);
        pivotSim.update(dt);

        inputs.rollerVelocityRotationsPerSec = rollerSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
        inputs.rollerAppliedVolts = rollerSim.getInputVoltage();
        inputs.rollerTorqueCurrent = rollerSim.getCurrentDrawAmps();
        inputs.rollerTemperatureFahrenheit = 70.0;

        inputs.pivotAngleRotations = pivotSim.getAngularPositionRotations();
        inputs.pivotVelocityRotationsPerSec = pivotSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
        inputs.pivotAppliedVolts = pivotSim.getInputVoltage();
        inputs.pivotTorqueCurrent = pivotSim.getCurrentDrawAmps();
        inputs.pivotTemperatureFahrenheit = 70.0;
    }

    @Override
    public void setVelocity(double velocityRotationsPerSec) {
        rollerFeedback.setSetpoint(velocityRotationsPerSec);
        desiredRollerVelocityRotationsPerSec = velocityRotationsPerSec;
        isRollerClosedLoop = true;
    }

    @Override
    public void setVoltage(double voltage) {
        rollerSim.setInputVoltage(voltage);
        isRollerClosedLoop = false;
    }

    @Override
    public void setPivotAngle(double angleRotations) {
        double clampedAngle = MathUtil.clamp(angleRotations,
            config.pivotMinAngleRotations,
            config.pivotMaxAngleRotations);
        pivotFeedback.setSetpoint(clampedAngle * (2 * Math.PI));
        isPivotClosedLoop = true;
    }

    @Override
    public void configureControlLoop(MotorControlLoopConfig config) {
        rollerFeedback.setP(config.kP());
        rollerFeedback.setI(config.kI());
        rollerFeedback.setD(config.kD());

        rollerFeedforward.setKs(config.kS());
        rollerFeedforward.setKv(config.kV());
        rollerFeedforward.setKa(config.kA());
    }

    @Override
    public void configurePivotControlLoop(MotorControlLoopConfig config) {
        pivotFeedback.setP(config.kP());
        pivotFeedback.setI(config.kI());
        pivotFeedback.setD(config.kD());
    }
}
