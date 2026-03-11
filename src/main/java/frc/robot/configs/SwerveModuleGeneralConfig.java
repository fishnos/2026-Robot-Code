package frc.robot.configs;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SwerveModuleGeneralConfig {
    public String canBusName;
    public double driveStatorCurrentLimit;
    public double drivePeakForwardTorqueCurrent;
    public double drivePeakReverseTorqueCurrent;

    public double driveKS;
    public double driveKV;
    public double driveKA;
    public double driveKP;
    public double driveKI;
    public double driveKD;

    public double driveMaxVelocityMetersPerSec;
    public boolean isDriveNeutralModeBrake;

    public double driveMotorToOutputShaftRatio;
    public double driveWheelRadiusMeters;

    public double steerStatorCurrentLimit;
    public double steerPeakForwardTorqueCurrent;
    public double steerPeakReverseTorqueCurrent;

    public double steerKS;
    public double steerKV;
    public double steerKA;
    public double steerKP;
    public double steerKI;
    public double steerKD;

    public boolean isSteerNeutralModeBrake;

    public double steerMotorToOutputShaftRatio;
    public double steerRotorToSensorRatio;

    public String steerCancoderFeedbackSensorSource;
    public String cancoderSensorDirection;
    public double cancoderAbsoluteSensorDiscontinuityPoint;

    public double driveMinWallCurrent;
    public double driveMaxWallVeloMetersPerSec;

    public FeedbackSensorSourceValue getSteerCancoderFeedbackSensorSource() {
        return FeedbackSensorSourceValue.valueOf(steerCancoderFeedbackSensorSource);
    }

    public SensorDirectionValue getCancoderSensorDirection() {
        return SensorDirectionValue.valueOf(cancoderSensorDirection);
    }
}
