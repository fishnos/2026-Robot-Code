package frc.robot.subsystems.swerve.gyro;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;
import frc.robot.configs.SwerveGyroConfig;
import frc.robot.lib.util.PhoenixUtil;
import frc.robot.subsystems.swerve.PhoenixOdometryThread;
import frc.robot.subsystems.swerve.SwerveDrive;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 gyro;

    private final StatusSignal<Angle> yawSignal;
    private final StatusSignal<Angle> rollSignal;
    private final StatusSignal<Angle> pitchSignal;
    private final StatusSignal<AngularVelocity> yawVelocitySignal;

    private final Queue<Double> odometryTimestampQueue;
    private final Queue<Double> yawPositionQueue;

    public GyroIOPigeon2(SwerveGyroConfig gyroConfig) {
        gyro = new Pigeon2(gyroConfig.canId, new CANBus(gyroConfig.canBusName));
        Pigeon2Configuration config = new Pigeon2Configuration();
        config.MountPose.MountPoseYaw = gyroConfig.mountPoseYaw;
        config.MountPose.MountPosePitch = gyroConfig.mountPosePitch;
        config.MountPose.MountPoseRoll = gyroConfig.mountPoseRoll;
        config.GyroTrim.GyroScalarZ = gyroConfig.gyroScalarZ;

        PhoenixUtil.tryUntilOk(5, () -> gyro.getConfigurator().apply(config, 0.25));

        yawSignal = gyro.getYaw().clone();
        rollSignal = gyro.getRoll().clone();
        pitchSignal = gyro.getPitch().clone();
        yawVelocitySignal = gyro.getAngularVelocityZWorld().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
            SwerveDrive.ODOMETRY_FREQUENCY,
            yawSignal,
            rollSignal,
            pitchSignal,
            yawVelocitySignal
        );

        odometryTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yawSignal.clone());

        gyro.optimizeBusUtilization();
    }

    @Override
    public synchronized void updateInputs(GyroIOInputs inputs) {
        BaseStatusSignal.refreshAll(yawSignal, rollSignal, pitchSignal, yawVelocitySignal);

        inputs.isConnected = true;

        double yawRadians = MathUtil.angleModulus(
            BaseStatusSignal.getLatencyCompensatedValue(yawSignal, yawVelocitySignal).in(Radians)
        );
        inputs.gyroOrientation = new Rotation3d(
            rollSignal.getValue().in(Radians),
            pitchSignal.getValue().in(Radians),
            yawRadians
        );
        inputs.yawVelocityRadPerSec = yawVelocitySignal.getValue().in(RadiansPerSecond);

        inputs.odometryTimestampsSeconds = odometryTimestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream().map((Double value) -> Rotation2d.fromDegrees(value)).toArray(Rotation2d[]::new);

        odometryTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    @Override
    public void resetGyro(Rotation2d yaw) {
        gyro.setYaw(yaw.getDegrees());
    }
}