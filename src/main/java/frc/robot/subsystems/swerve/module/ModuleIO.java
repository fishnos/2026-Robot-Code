package frc.robot.subsystems.swerve.module;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public double drivePositionMeters = 0;
        public double driveVelocityMetersPerSec = 0;
        public double driveAppliedVolts = 0;

        public Rotation2d steerPosition = new Rotation2d();
        public double steerVelocityRadPerSec = 0;
        public double steerAppliedVolts = 0;

        public Rotation2d steerEncoderAbsolutePosition = new Rotation2d();
        public Rotation2d steerEncoderPosition = new Rotation2d();

        public double driveTorqueCurrent = 0;
        public double driveTemperatureFahrenheit = 0;

        public double steerTorqueCurrent = 0;
        public double steerTemperatureFahrenheit = 0;

        public double[] odometryTimestampsSeconds = new double[] {};
        public double[] odometryDrivePositionsMeters = new double[] {};
        public Rotation2d[] odometrySteerPositions = new Rotation2d[] {};
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setState(SwerveModuleState state) {}
    public default void setSteerTorqueCurrentFOC(double torqueCurrentFOC, double driveVelocityMetersPerSec) {}
    public default void setDriveTorqueCurrentFOC(double torqueCurrentFOC, Rotation2d steerAngle) {}
    public default void configureDriveControlLoop(MotorControlLoopConfig config) {}
    public default void configureSteerControlLoop(MotorControlLoopConfig config) {}
    public default void setWheelCoast(boolean isCoast) {}
    public default void enableDriveEStop() {}
    public default void disableDriveEStop() {}
    public default void enableSteerEStop() {}
    public default void disableSteerEStop() {}
}
