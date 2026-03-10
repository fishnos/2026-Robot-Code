package frc.robot.configs;

public class KickerConfig {
    public String canBusName;

    public int kickerCanId;
    public boolean isKickerInverted;

    public double kickerStatorCurrentLimit;
    public double kickerPeakForwardTorqueCurrent;
    public double kickerPeakReverseTorqueCurrent;

    public double kickerKS;
    public double kickerKV;
    public double kickerKA;
    public double kickerKP;
    public double kickerKI;
    public double kickerKD;

    public double kickerMotorToOutputShaftRatio;

    public double feedingVelocityRPS;
    public double kickingVelocityRPS;

    public double kickerVelocityToleranceRPS;
}
