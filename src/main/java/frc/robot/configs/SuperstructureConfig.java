package frc.robot.configs;

public class SuperstructureConfig {
    public double shotDurationSeconds = 0.3;
    public double alternatingIntakeToggleSeconds = 1.0;
    public double ballsPerSecond = 9.5;

    public double turretRotationBufferDeg = 20.0;
    public double maxTranslationalVelocityDuringShotMetersPerSec = 2.5;
    public double maxAngularVelocityDuringShotRadPerSec = 3.0;
    public double shotImpactToleranceMeters = 0.3;
    public double lastInRangeShotMaxAgeSeconds = 1.0;
    public double bumpMaxVelocityMetersPerSec = 1.8;
    public double bumpSnapAngleDegrees = 45.0;
    public double passHubBlockerRadiusMeters = 0.12;

    public double latencyCompensationSeconds = 0.0;
    public double ballisticsSimulationStepSeconds = 0.002;
}
