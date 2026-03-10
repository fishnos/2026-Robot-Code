package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.configs.SwerveModuleGeneralConfig;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;

public class ModuleIOSim implements ModuleIO {
    private final DCMotor driveMotorModel = DCMotor.getKrakenX60Foc(1);
    private final DCMotor turnMotorModel = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim driveSim =
        new DCMotorSim( 
            LinearSystemId.createDCMotorSystem(driveMotorModel, (2.8087 * 0.0194 * 0.0485614385) / 6.12, 6.12), // J = (kA_linear * Kt * r) / G
            driveMotorModel
        );
    private final DCMotorSim steerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnMotorModel,  0.00015, 21.428), // magic number because steer is not important
            turnMotorModel
        );


    private PIDController driveFeedback = new PIDController(0, 0.0, 0.0);
    private PIDController steerFeedback = new PIDController(25, 0.0, 0.0);

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 2.44, 0.1);

    private boolean isSteerClosedLoop = true;
    private boolean isDriveClosedLoop = true;

    private SwerveModuleState lastDesiredState = new SwerveModuleState();

    private double lastTimeInputs = Timer.getTimestamp();
    
    private final int moduleID;
    public ModuleIOSim(SwerveModuleGeneralConfig config, int moduleID) {
        this.moduleID = moduleID;
        steerFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        double dt = Timer.getTimestamp() - lastTimeInputs;
        lastTimeInputs = Timer.getTimestamp();

        if (isDriveClosedLoop) {
            driveSim.setInputVoltage(
                MathUtil.clamp(
                    driveFeedforward.calculate(lastDesiredState.speedMetersPerSecond) +
                    driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec() * 0.0485614385), // wheel radius in meters
                    -12,
                    12
                )
            );
        }

        if (isSteerClosedLoop) {
            steerSim.setInputVoltage(
                MathUtil.clamp(
                    steerFeedback.calculate(steerSim.getAngularPositionRad()),
                    -12,
                    12
                )
            );
        }

        steerSim.update(dt);
        driveSim.update(dt);
        
        inputs.drivePositionMeters = driveSim.getAngularPositionRad() * 0.0485614385; // wheel radius in meters
        inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRadPerSec() * 0.0485614385;
        inputs.driveAppliedVolts = driveSim.getInputVoltage();

        inputs.steerPosition = new Rotation2d(steerSim.getAngularPosition());
        inputs.steerVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
        inputs.steerAppliedVolts = steerSim.getInputVoltage();

        inputs.steerEncoderAbsolutePosition = inputs.steerPosition;
        inputs.steerEncoderPosition = inputs.steerPosition;

        inputs.driveTorqueCurrent = driveSim.getCurrentDrawAmps();
        inputs.steerTorqueCurrent = steerSim.getCurrentDrawAmps();

        inputs.odometryTimestampsSeconds = new double[] {Timer.getTimestamp()};
        inputs.odometryDrivePositionsMeters = new double[] {inputs.drivePositionMeters};
        inputs.odometrySteerPositions = new Rotation2d[] {inputs.steerPosition};
    }

    @Override
    public void setState(SwerveModuleState state) {
        driveFeedback.setSetpoint(state.speedMetersPerSecond);
        steerFeedback.setSetpoint(state.angle.getRadians());

        isDriveClosedLoop = true;
        isSteerClosedLoop = true;

        lastDesiredState = state;
    }

    @Override
    public void setSteerTorqueCurrentFOC(double torqueCurrentFOC, double driveVelocityMetersPerSec) {
        // In sim, treat torqueCurrentFOC as voltage for simplicity
        steerSim.setInputVoltage(torqueCurrentFOC);

        isDriveClosedLoop = false;
        isSteerClosedLoop = true;
    }

    @Override
    public void setDriveTorqueCurrentFOC(double torqueCurrentFOC, Rotation2d steerAngle) {
        // In sim, treat torqueCurrentFOC as voltage for simplicity
        driveSim.setInputVoltage(torqueCurrentFOC);

        isDriveClosedLoop = false;
        isSteerClosedLoop = true;
    }

    @Override
    public void configureDriveControlLoop(MotorControlLoopConfig config) {
        // No-op in sim
    }

    @Override
    public void configureSteerControlLoop(MotorControlLoopConfig config) {
        // No-op in sim
    }
}
