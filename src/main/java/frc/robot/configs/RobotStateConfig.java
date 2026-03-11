package frc.robot.configs;

public class RobotStateConfig {
    public double poseBufferSizeSeconds = 2.0;
    public double odomTranslationDevBase;
    public double odomRotationDevBase;
    public double visionTranslationDevBase;
    public double visionRotationDevBase;
    public double gyroResetTimeoutSeconds = 1.0;
    public double gyroResetMaxVisionRotationStdDev = 300.0;
    public double maxTiltAngleDegrees = 15.0;
}
