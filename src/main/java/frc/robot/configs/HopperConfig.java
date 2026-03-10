package frc.robot.configs;

public class HopperConfig {
    public String canBusName;

    public int hopperCanId;
    public boolean isHopperInverted;

    public double hopperStatorCurrentLimit;
    public double hopperPeakForwardTorqueCurrent;
    public double hopperPeakReverseTorqueCurrent;

    public double hopperKS;
    public double hopperKV;
    public double hopperKA;
    public double hopperKP;
    public double hopperKI;
    public double hopperKD;

    public double hopperMotorToOutputShaftRatio;

    public double feedingVelocityRPS;
    public double reverseVelocityRPS;

    public double hopperVelocityToleranceRPS;
}
