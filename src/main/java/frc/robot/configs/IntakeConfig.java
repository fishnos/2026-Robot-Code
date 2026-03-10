package frc.robot.configs;

public class IntakeConfig {
    public String canBusName;

    public int rollerCanId;
    public boolean isRollerInverted;

    public double rollerStatorCurrentLimit;
    public double rollerPeakForwardTorqueCurrent;
    public double rollerPeakReverseTorqueCurrent;

    public double rollerKS;
    public double rollerKV;
    public double rollerKA;
    public double rollerKP;
    public double rollerKI;
    public double rollerKD;

    public double rollerMotorToOutputShaftRatio;

    public double intakingVelocityRPS;
    public double outtakingVelocityRPS;

    public double rollerVelocityToleranceRPS;

    public int pivotCanId;
    public boolean isPivotInverted;

    public double pivotStatorCurrentLimit;
    public double pivotPeakForwardTorqueCurrent;
    public double pivotPeakReverseTorqueCurrent;

    public double pivotKS;
    public double pivotKV;
    public double pivotKA;
    public double pivotKP;
    public double pivotKI;
    public double pivotKD;

    public double pivotMotorToOutputShaftRatio;

    public double pivotStartingAngleRotations;
    public double pivotMinAngleRotations;
    public double pivotMaxAngleRotations;

    public double pivotAngleToleranceRotations;
    public double pivotUpAngleRotations;
    public double pivotDownAngleRotations;
}
