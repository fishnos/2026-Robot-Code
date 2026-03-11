package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public double rollerVelocityRotationsPerSec = 0;
        public double rollerAppliedVolts = 0;
        public double rollerTorqueCurrent = 0;
        public double rollerTemperatureFahrenheit = 0;

        public double pivotAngleRotations = 0;
        public double pivotVelocityRotationsPerSec = 0;
        public double pivotAppliedVolts = 0;
        public double pivotTorqueCurrent = 0;
        public double pivotTemperatureFahrenheit = 0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    public default void setVelocity(double velocityRotationsPerSec) {}
    public default void setVoltage(double voltage) {}
    public default void setPivotAngle(double angleRotations) {}
    public default void configureControlLoop(MotorControlLoopConfig config) {}
    public default void configurePivotControlLoop(MotorControlLoopConfig config) {}
    public default void enableRollerEStop() {}
    public default void disableRollerEStop() {}
    public default void enablePivotEStop() {}
    public default void disablePivotEStop() {}
}
