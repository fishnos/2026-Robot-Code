package frc.robot.configs;

public class ClimberConfig {
    public String canBusName;

    public int climberCanId;
    public boolean isClimberInverted;

    public double climberStatorCurrentLimit;
    public double climberPeakForwardTorqueCurrent;
    public double climberPeakReverseTorqueCurrent;

    public double climberKS;
    public double climberKV;
    public double climberKA;
    public double climberKP;
    public double climberKI;
    public double climberKD;

    public double climberMotorToOutputShaftRatio;

    public double climberStartingPositionRotations;
    public double climberHomePositionRotations;
    public double climberReadyPositionRotations;
    public double climberLiftedPositionRotations;

    public double climberMinPositionRotations;
    public double climberMaxPositionRotations;

    public double climberDefaultProfileMaxVelocityRotationsPerSec;
    public double climberDefaultProfileMaxAccelerationRotationsPerSec2;
    public double climberClimbingProfileMaxVelocityRotationsPerSec;
    public double climberClimbingProfileMaxAccelerationRotationsPerSec2;

    public double climberPositionToleranceRotations;
}
