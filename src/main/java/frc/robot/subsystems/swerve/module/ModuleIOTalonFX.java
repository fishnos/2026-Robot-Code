package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.configs.SwerveModuleGeneralConfig;
import frc.robot.configs.SwerveModuleSpecificConfig;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;
import frc.robot.lib.util.RebelUtil;
import frc.robot.subsystems.swerve.PhoenixOdometryThread;
import frc.robot.lib.util.PhoenixUtil;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ModuleIOTalonFX implements ModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double>  steerPositionQueue;

    private final StatusSignal<Angle> drivePositionStatusSignal;
    private final StatusSignal<AngularVelocity> driveVelocityStatusSignal;
    private final StatusSignal<Voltage> driveMotorVoltage;

    private final StatusSignal<Angle> steerPositionStatusSignal;
    private final StatusSignal<AngularVelocity> steerVelocityStatusSignal;
    private final StatusSignal<Voltage> steerMotorVoltage;

    private final StatusSignal<Angle> steerEncoderPositionStatusSignal;
    private final StatusSignal<Angle> steerEncoderAbsolutePosition;

    private final StatusSignal<Current> driveTorqueCurrent;
    private final StatusSignal<Temperature> driveTemperature;

    private final StatusSignal<Current> steerTorqueCurrent;
    private final StatusSignal<Temperature> steerTemperature;

    // private final VelocityTorqueCurrentFOC driveMotorRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final VelocityVoltage driveMotorRequest = new VelocityVoltage(0).withSlot(0);

    // private final MotionMagicExpoTorqueCurrentFOC steerMotorRequest = new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);
    private final PositionVoltage steerMotorRequest = new PositionVoltage(0).withSlot(0);

    private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0);

    private final SwerveModuleGeneralConfig generalConfig;
    private final TalonFXConfiguration driveConfig;
    private final TalonFXConfiguration steerConfig;

    private Rotation2d lastSteerAngleRad = new Rotation2d();
    private SwerveModuleState lastRequestedState = new SwerveModuleState();
    private double lastRequestedStateTime = Timer.getFPGATimestamp();

    public ModuleIOTalonFX(SwerveModuleGeneralConfig generalConfig, SwerveModuleSpecificConfig specificConfig) {
        this.generalConfig = generalConfig;

        driveConfig = new TalonFXConfiguration();

        driveConfig.Slot0.kP = generalConfig.driveKP;
        driveConfig.Slot0.kI = generalConfig.driveKI;
        driveConfig.Slot0.kD = generalConfig.driveKD;
        driveConfig.Slot0.kS = generalConfig.driveKS;
        driveConfig.Slot0.kV = generalConfig.driveKV;
        driveConfig.Slot0.kA = generalConfig.driveKA;
        driveConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        // Cancoder + encoder
        driveConfig.ClosedLoopGeneral.ContinuousWrap = false;
        driveConfig.Feedback.SensorToMechanismRatio = 
            generalConfig.driveMotorToOutputShaftRatio /
            (generalConfig.driveWheelRadiusMeters * 2 * Math.PI);

        driveConfig.MotorOutput.NeutralMode = 
            generalConfig.isDriveNeutralModeBrake ? 
                NeutralModeValue.Brake : 
                NeutralModeValue.Coast;

        driveConfig.MotorOutput.Inverted = 
            specificConfig.isDriveInverted ?
                InvertedValue.Clockwise_Positive :
                InvertedValue.CounterClockwise_Positive;

        // Current and torque limiting
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = generalConfig.driveStatorCurrentLimit;

        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = generalConfig.drivePeakForwardTorqueCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = generalConfig.drivePeakReverseTorqueCurrent;

        driveConfig.FutureProofConfigs = true;
        
        CANBus canBus = new CANBus(generalConfig.canBusName);
        driveMotor = new TalonFX(specificConfig.driveCanId, canBus);
        PhoenixUtil.tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));

        // ABS encoder
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = generalConfig.cancoderAbsoluteSensorDiscontinuityPoint;
        encoderConfig.MagnetSensor.SensorDirection = generalConfig.getCancoderSensorDirection();
        encoderConfig.MagnetSensor.withMagnetOffset(specificConfig.cancoderOffsetRotations);

        encoderConfig.FutureProofConfigs = true;

        steerEncoder = new CANcoder(specificConfig.cancoderCanId, canBus);
        PhoenixUtil.tryUntilOk(5, () -> steerEncoder.getConfigurator().apply(encoderConfig, 0.25));

        // Steer motor
        steerConfig = new TalonFXConfiguration();

        steerConfig.Slot0.kP = generalConfig.steerKP;
        steerConfig.Slot0.kI = generalConfig.steerKI;
        steerConfig.Slot0.kD = generalConfig.steerKD;
        steerConfig.Slot0.kS = generalConfig.steerKS;
        steerConfig.Slot0.kV = generalConfig.steerKV;
        steerConfig.Slot0.kA = generalConfig.steerKA;
        steerConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        steerConfig.MotorOutput.NeutralMode = 
            generalConfig.isSteerNeutralModeBrake ? 
                NeutralModeValue.Brake : 
                NeutralModeValue.Coast;

        steerConfig.MotorOutput.Inverted = 
            specificConfig.isSteerInverted ?
                InvertedValue.Clockwise_Positive :
                InvertedValue.CounterClockwise_Positive;
                
        // Cancoder + encoder
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        steerConfig.Feedback.FeedbackRemoteSensorID = specificConfig.cancoderCanId;
        steerConfig.Feedback.FeedbackSensorSource = generalConfig.getSteerCancoderFeedbackSensorSource();
        steerConfig.Feedback.SensorToMechanismRatio = 1;
        steerConfig.Feedback.RotorToSensorRatio = generalConfig.steerRotorToSensorRatio;

        // current and torque limiting
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfig.CurrentLimits.StatorCurrentLimit = generalConfig.steerStatorCurrentLimit;

        steerConfig.TorqueCurrent.PeakForwardTorqueCurrent = generalConfig.steerPeakForwardTorqueCurrent;
        steerConfig.TorqueCurrent.PeakReverseTorqueCurrent = generalConfig.steerPeakReverseTorqueCurrent;

        steerConfig.FutureProofConfigs = true;

        steerMotor = new TalonFX(specificConfig.steerCanId, canBus);
        PhoenixUtil.tryUntilOk(5, () -> steerMotor.getConfigurator().apply(steerConfig, 0.25));

        // status signals
        driveTorqueCurrent = driveMotor.getTorqueCurrent().clone();
        driveTemperature = driveMotor.getDeviceTemp().clone();
        driveMotorVoltage = driveMotor.getMotorVoltage().clone();

        steerTorqueCurrent = steerMotor.getTorqueCurrent().clone();
        steerTemperature = steerMotor.getDeviceTemp().clone();
        steerMotorVoltage = steerMotor.getMotorVoltage().clone();

        steerEncoderAbsolutePosition = steerEncoder.getAbsolutePosition().clone();
        steerEncoderPositionStatusSignal = steerEncoder.getPosition().clone();

        drivePositionStatusSignal = driveMotor.getPosition().clone();
        driveVelocityStatusSignal = driveMotor.getVelocity().clone();

        steerPositionStatusSignal = steerMotor.getPosition().clone();
        steerVelocityStatusSignal = steerMotor.getVelocity().clone();

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(drivePositionStatusSignal.clone());
        steerPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(steerPositionStatusSignal.clone());

        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            driveTorqueCurrent,
            driveTemperature,
            driveMotorVoltage,

            steerTorqueCurrent,
            steerTemperature,
            steerMotorVoltage,

            steerEncoderAbsolutePosition,
            steerEncoderPositionStatusSignal
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            SwerveDrive.ODOMETRY_FREQUENCY, 
            drivePositionStatusSignal, 
            steerPositionStatusSignal,

            driveVelocityStatusSignal,
            steerVelocityStatusSignal
        );

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
        steerEncoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            driveTorqueCurrent,
            driveTemperature,
            driveMotorVoltage,

            steerTorqueCurrent,
            steerTemperature,
            steerMotorVoltage,

            steerEncoderAbsolutePosition,
            steerEncoderPositionStatusSignal,

            drivePositionStatusSignal,
            driveVelocityStatusSignal,

            steerPositionStatusSignal,
            steerVelocityStatusSignal
        );

        inputs.drivePositionMeters = BaseStatusSignal.getLatencyCompensatedValue(drivePositionStatusSignal, driveVelocityStatusSignal).in(Rotation);
        inputs.driveVelocityMetersPerSec = driveVelocityStatusSignal.getValue().in(RotationsPerSecond);
        inputs.driveAppliedVolts = driveMotorVoltage.getValue().in(Volts);

        inputs.steerPosition = new Rotation2d(BaseStatusSignal.getLatencyCompensatedValue(steerPositionStatusSignal, steerVelocityStatusSignal).in(Radians));
        inputs.steerVelocityRadPerSec = steerVelocityStatusSignal.getValue().in(RadiansPerSecond);
        inputs.steerAppliedVolts = steerMotorVoltage.getValue().in(Volts);

        inputs.steerEncoderAbsolutePosition = new Rotation2d(steerEncoderAbsolutePosition.getValue().in(Radians));
        inputs.steerEncoderPosition = new Rotation2d(steerEncoderPositionStatusSignal.getValue().in(Radians));

        inputs.driveTorqueCurrent = driveTorqueCurrent.getValue().in(Amps);
        inputs.driveTemperatureFahrenheit = driveTemperature.getValue().in(Fahrenheit);

        inputs.steerTorqueCurrent = steerTorqueCurrent.getValue().in(Amps);
        inputs.steerTemperatureFahrenheit = steerTemperature.getValue().in(Fahrenheit);

        inputs.odometryTimestampsSeconds = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.odometryDrivePositionsMeters = drivePositionQueue.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.odometrySteerPositions = steerPositionQueue.stream().map((Double value) -> Rotation2d.fromRotations(value)).toArray(Rotation2d[]::new);

        timestampQueue.clear();
        drivePositionQueue.clear();
        steerPositionQueue.clear();

        lastSteerAngleRad = new Rotation2d(inputs.steerPosition.getRadians());
    }

    @Override
    public void setState(SwerveModuleState state) {
        driveMotor.setControl(driveMotorRequest.withVelocity(
                RebelUtil.constrain(
                    state.speedMetersPerSecond,
                    -generalConfig.driveMaxVelocityMetersPerSec,
                    generalConfig.driveMaxVelocityMetersPerSec
                ) * state.angle.minus(lastSteerAngleRad).getCos()
            ).withAcceleration((state.speedMetersPerSecond - lastRequestedState.speedMetersPerSecond) / (Timer.getFPGATimestamp() - lastRequestedStateTime))
        );
        
        steerMotor.setControl(
            steerMotorRequest.withPosition(
                state.angle.getRotations()
            )
        );

        lastRequestedState = state;
        lastRequestedStateTime = Timer.getFPGATimestamp();
    }

    @Override
    public void setSteerTorqueCurrentFOC(double torqueCurrentFOC, double driveVelocityMetersPerSec) {
        // Set steer motor with torque FOC, optionally using drive velocity for feedforward if needed
        steerMotor.setControl(
            torqueCurrentFOCRequest.withOutput(torqueCurrentFOC)
        );

        driveMotor.setControl(driveMotorRequest.withVelocity(
                RebelUtil.constrain(
                    driveVelocityMetersPerSec,
                    -generalConfig.driveMaxVelocityMetersPerSec,
                    generalConfig.driveMaxVelocityMetersPerSec
                )
            )
        );
    }

    @Override
    public void setDriveTorqueCurrentFOC(double torqueCurrentFOC, Rotation2d steerAngle) {
        // Set drive motor with torque FOC, optionally using steer angle for feedforward if needed
        driveMotor.setControl(
            torqueCurrentFOCRequest.withOutput(torqueCurrentFOC)
        );

        steerMotor.setControl(
            steerMotorRequest.withPosition(
                steerAngle.getRotations()
            )
        );
    }

    @Override
    public void configureDriveControlLoop(MotorControlLoopConfig config) {
        driveConfig.Slot0.kP = config.kP();
        driveConfig.Slot0.kI = config.kI();
        driveConfig.Slot0.kD = config.kD();
        driveConfig.Slot0.kS = config.kS();
        driveConfig.Slot0.kV = config.kV();
        driveConfig.Slot0.kA = config.kA();

        PhoenixUtil.tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));
    }

    @Override
    public void configureSteerControlLoop(MotorControlLoopConfig config) {
        steerConfig.Slot0.kP = config.kP();
        steerConfig.Slot0.kI = config.kI();
        steerConfig.Slot0.kD = config.kD();
        steerConfig.Slot0.kS = config.kS();
        steerConfig.Slot0.kV = config.kV();
        steerConfig.Slot0.kA = config.kA();

        PhoenixUtil.tryUntilOk(5, () -> steerMotor.getConfigurator().apply(steerConfig, 0.25));
    }

    @Override
    public void setWheelCoast(boolean isCoast) {
        driveMotor.setNeutralMode(isCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        steerMotor.setNeutralMode(isCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }
}