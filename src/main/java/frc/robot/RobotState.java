package frc.robot;

import edu.wpi.first.math.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.configs.RobotStateConfig;
import frc.robot.configs.SwerveConfig;
import frc.robot.configs.SwerveDrivetrainConfig;
import frc.robot.lib.util.ConfigLoader;
import frc.robot.subsystems.swerve.SwerveDrive;

import java.util.NoSuchElementException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// TODO: Make it so that if it is tracking / shooting, the center of rotation for the robot is around the shooter's position
public class RobotState {
    private static final double OBSERVATION_LOG_PERIOD_SECONDS = 0.1;

    private static RobotState instance;
    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }
  
    public record OdometryObservation(
        double timestampsSeconds,
        boolean isGyroConnected,
        SwerveModulePosition[] modulePositions, 
        SwerveModuleState[] moduleStates,
        Rotation3d gyroOrientation,
        double yawVelocityRadPerSec
    ) {}


    public record VisionObservation(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {}


    private final TimeInterpolatableBuffer<Pose2d> poseBuffer;

    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    private double lastEstimatedPoseUpdateTime = 0;
    private int visionObservationsAccepted = 0;
    private int visionObservationsRejected = 0;
    private double lastOdometryObservationLogTimestampSeconds = Double.NEGATIVE_INFINITY;
    private double lastVisionObservationLogTimestampSeconds = Double.NEGATIVE_INFINITY;

    private double lastGyroResetTime = Timer.getTimestamp();

    private Rotation3d lastGyroAngle = new Rotation3d();

    // Odometry
    private final SwerveDriveKinematics kinematics;
    private SwerveModulePosition[] lastWheelPositions = {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    private double lastYawVelocityRadPerSec = 0;
    private ChassisSpeeds lastRobotRelativeSpeeds = new ChassisSpeeds();
    private double accumulatedYawRad = 0.0;
    private double lastWrappedYawRad = 0.0;
    private boolean hasYawSample = false;

    private final SwerveDrivetrainConfig drivetrainConfig;
    private final RobotStateConfig robotStateConfig;

    private RobotState() {
        SwerveConfig swerveConfig = ConfigLoader.load("swerve", SwerveConfig.class);
        drivetrainConfig = swerveConfig.drivetrain;
        robotStateConfig = ConfigLoader.load("robotState", RobotStateConfig.class);
        poseBuffer = TimeInterpolatableBuffer.createBuffer(robotStateConfig.poseBufferSizeSeconds);

        kinematics = new SwerveDriveKinematics(
            drivetrainConfig.getFrontLeftPositionMeters(),
            drivetrainConfig.getFrontRightPositionMeters(),
            drivetrainConfig.getBackLeftPositionMeters(),
            drivetrainConfig.getBackRightPositionMeters()
        ); 

        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            new Rotation2d(), 
            lastWheelPositions, 
            new Pose2d(),
            VecBuilder.fill(
                robotStateConfig.odomTranslationDevBase,
                robotStateConfig.odomTranslationDevBase,
                robotStateConfig.odomRotationDevBase

            ),
            VecBuilder.fill(
                robotStateConfig.visionTranslationDevBase,
                robotStateConfig.visionTranslationDevBase,
                robotStateConfig.visionRotationDevBase
            )
        );  
    }   

    /** Add odometry observation */
    public void addOdometryObservation(OdometryObservation observation) {
        observation = new OdometryObservation(
            observation.timestampsSeconds(),
            observation.isGyroConnected(),
            observation.modulePositions().clone(),
            observation.moduleStates().clone(),
            observation.gyroOrientation(),
            observation.yawVelocityRadPerSec()
        );
        
        boolean shouldLogObservation = shouldLogOdometryObservation();
        if (shouldLogObservation) {
            Logger.recordOutput("RobotState/odometry/timestamp", observation.timestampsSeconds());
            Logger.recordOutput("RobotState/odometry/isGyroConnected", observation.isGyroConnected());
            Logger.recordOutput("RobotState/odometry/modulePositions", observation.modulePositions());
            Logger.recordOutput("RobotState/odometry/moduleStates", observation.moduleStates());
            Logger.recordOutput("RobotState/odometry/gyroOrientation", observation.gyroOrientation());
            Logger.recordOutput("RobotState/odometry/yawVelocityRadPerSec", observation.yawVelocityRadPerSec());
        }

        // update robotState member variables
        lastRobotRelativeSpeeds = kinematics.toChassisSpeeds(observation.moduleStates);
        lastRobotRelativeSpeeds.omegaRadiansPerSecond = observation.isGyroConnected ? observation.yawVelocityRadPerSec() : lastRobotRelativeSpeeds.omegaRadiansPerSecond;
        lastYawVelocityRadPerSec = observation.isGyroConnected ? observation.yawVelocityRadPerSec() : lastRobotRelativeSpeeds.omegaRadiansPerSecond;

        Rotation3d gyroOrientation = observation.isGyroConnected() ?
            observation.gyroOrientation() :
            new Rotation3d(
                0.0,
                0.0,
                swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getRadians()
            );
        lastGyroAngle = gyroOrientation;

        // Reject odometry if robot is tilted too much
        boolean isTilted = getAngleToFloor().getDegrees() > robotStateConfig.maxTiltAngleDegrees;
        if (shouldLogObservation) {
            Logger.recordOutput("RobotState/odometry/isTilted", isTilted);
        }
        if (isTilted) {
            // use old wheel positions
            swerveDrivePoseEstimator.updateWithTime(
                observation.timestampsSeconds(), 
                observation.isGyroConnected() ? 
                    gyroOrientation.toRotation2d() : 
                    new Rotation2d(
                        swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getRadians() + 
                        kinematics.toTwist2d(lastWheelPositions, observation.modulePositions()).dtheta
                    ),
                lastWheelPositions
            );  
        } else {
            swerveDrivePoseEstimator.updateWithTime(
                observation.timestampsSeconds(), 
                observation.isGyroConnected() ? 
                    gyroOrientation.toRotation2d() : 
                    new Rotation2d(
                        swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getRadians() + 
                        kinematics.toTwist2d(lastWheelPositions, observation.modulePositions()).dtheta
                    ),
                observation.modulePositions()
            );    

            lastWheelPositions = observation.modulePositions();
        }
        
        lastEstimatedPoseUpdateTime = observation.timestampsSeconds(); 
        // Add pose to buffer at timestamp
        poseBuffer.addSample(lastEstimatedPoseUpdateTime, swerveDrivePoseEstimator.getEstimatedPosition()); 
        updateAccumulatedYawFromEstimatedPose();
    }

    public void addVisionObservation(VisionObservation observation) {
        boolean shouldLogObservation = shouldLogVisionObservation();
        // If measurement is old enough to be outside the pose buffer's timespan, skip.
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - robotStateConfig.poseBufferSizeSeconds > observation.timestampSeconds()) {
                incrementVisionRejectedCount();
                if (shouldLogObservation) {
                    logVisionObservationMetrics(null, observation.timestampSeconds(), false);
                }
                return;
            }
        } 
        
        catch (NoSuchElementException ex) {
            incrementVisionRejectedCount();
            if (shouldLogObservation) {
                logVisionObservationMetrics(null, observation.timestampSeconds(), false);
            }
            return;
        }

        visionObservationsAccepted++;

        if (shouldLogObservation) {
            logVisionObservationMetrics(observation, observation.timestampSeconds(), true);
        }

        boolean didResetGyro = false;
        if (DriverStation.isDisabled()
            && Timer.getTimestamp() - lastGyroResetTime > robotStateConfig.gyroResetTimeoutSeconds
            && observation.visionMeasurementStdDevs().get(2,0) < robotStateConfig.gyroResetMaxVisionRotationStdDev) {
                resetPose(new Pose2d(getEstimatedPose().getTranslation(), observation.visionRobotPoseMeters().getRotation()));
                lastGyroResetTime = Timer.getTimestamp();
                didResetGyro = true;
        }
        if (shouldLogObservation || didResetGyro) {
            logGyroResetStatus(
                didResetGyro,
                didResetGyro ? observation.visionRobotPoseMeters().getRotation() : null
            );
        }

        swerveDrivePoseEstimator.addVisionMeasurement(observation.visionRobotPoseMeters(), observation.timestampSeconds(), observation.visionMeasurementStdDevs());
        lastEstimatedPoseUpdateTime = Timer.getTimestamp();
    }

    /**
     * Reset estimated pose and odometry pose to pose <br>
     * Clear pose buffer
     */
    public void resetPose(Pose2d initialPose) {
        SwerveDrive.getInstance().resetGyro(initialPose.getRotation());
        swerveDrivePoseEstimator.resetPosition(initialPose.getRotation(), lastWheelPositions, initialPose);
        accumulatedYawRad = initialPose.getRotation().getRadians();
        lastWrappedYawRad = initialPose.getRotation().getRadians();
        hasYawSample = true;

        poseBuffer.clear();
    }

    public void zeroGyro() {
        resetPose(new Pose2d(getEstimatedPose().getTranslation(), new Rotation2d()));
    }

    @AutoLogOutput(key = "RobotState/estimatedPose")
    public Pose2d getEstimatedPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public double getYawVelocityRadPerSec() {
        return lastYawVelocityRadPerSec;
    }

    private boolean shouldLogOdometryObservation() {
        double now = Timer.getTimestamp();
        if (now - lastOdometryObservationLogTimestampSeconds >= OBSERVATION_LOG_PERIOD_SECONDS) {
            lastOdometryObservationLogTimestampSeconds = now;
            return true;
        }
        return false;
    }

    private boolean shouldLogVisionObservation() {
        double now = Timer.getTimestamp();
        if (now - lastVisionObservationLogTimestampSeconds >= OBSERVATION_LOG_PERIOD_SECONDS) {
            lastVisionObservationLogTimestampSeconds = now;
            return true;
        }
        return false;
    }

    private void logVisionObservationMetrics(
        VisionObservation observation,
        double observationTimestampSeconds,
        boolean includeObservationDetails
    ) {
        if (includeObservationDetails && observation != null) {
            Logger.recordOutput("RobotState/vision/stdDevTranslation", observation.visionMeasurementStdDevs().get(0,0));
            Logger.recordOutput("RobotState/vision/stdDevRotation", observation.visionMeasurementStdDevs().get(2,0));
            Logger.recordOutput("RobotState/vision/visionPose", observation.visionRobotPoseMeters());
        }
        Logger.recordOutput("RobotState/vision/visionObservationsAccepted", visionObservationsAccepted);
        Logger.recordOutput("RobotState/vision/visionObservationsRejected", visionObservationsRejected);
        Logger.recordOutput("RobotState/vision/visionLatency", Timer.getFPGATimestamp() - observationTimestampSeconds);
    }

    private void incrementVisionRejectedCount() {
        visionObservationsRejected++;
    }

    private void logGyroResetStatus(boolean didReset, Rotation2d rotation) {
        Logger.recordOutput("RobotState/vision/gyroReset", didReset);
        if (didReset && rotation != null) {
            Logger.recordOutput("RobotState/vision/gyroRestRotation", rotation);
        }
    }

    @AutoLogOutput(key = "RobotState/accumulatedYaw")
    public Rotation2d getAccumulatedYaw() {
        return Rotation2d.fromRadians(accumulatedYawRad);
    }

    @AutoLogOutput(key = "RobotState/accumulatedYawRad")
    public double getAccumulatedYawRadians() {
        return accumulatedYawRad;
    }

    @AutoLogOutput(key = "RobotState/accumulatedYawDeg")
    public double getAccumulatedYawDegrees() {
        return Math.toDegrees(accumulatedYawRad);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return lastRobotRelativeSpeeds;
    }

    public Rotation3d getLastOrientation() {
        return new Rotation3d(
            lastGyroAngle.getX(), 
            lastGyroAngle.getY(), 
            getEstimatedPose().getRotation().getRadians() // pose estim offsets for yaw taken into account
        );
    }

    @AutoLogOutput(key = "RobotState/angleToFloor")
    public Rotation2d getAngleToFloor() {
        double roll = lastGyroAngle.getX();
        double pitch = lastGyroAngle.getY();
        double cosine = Math.cos(roll) * Math.cos(pitch);
        return new Rotation2d(Math.acos(MathUtil.clamp(cosine, -1.0, 1.0)));
    }

    @AutoLogOutput(key = "RobotState/fieldRelativeSpeeds")
    public ChassisSpeeds getFieldRelativeSpeeds() { 
        return ChassisSpeeds.fromRobotRelativeSpeeds(lastRobotRelativeSpeeds, getEstimatedPose().getRotation());
    }

    public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
        return getEstimatedPose()
            .transformBy(
                new Transform2d(
                    lastRobotRelativeSpeeds.vxMetersPerSecond * translationLookaheadS,
                    lastRobotRelativeSpeeds.vyMetersPerSecond * translationLookaheadS,
                    Rotation2d.fromRadians(lastRobotRelativeSpeeds.omegaRadiansPerSecond * rotationLookaheadS)
                )
            );
    }

    public Pose2d getPredictedPose(double timestamp) {
        return getPredictedPose(timestamp - lastEstimatedPoseUpdateTime, timestamp - lastEstimatedPoseUpdateTime);
    }

    private void updateAccumulatedYawFromEstimatedPose() {
        double wrappedYawRad = getEstimatedPose().getRotation().getRadians();
        if (!hasYawSample) {
            accumulatedYawRad = wrappedYawRad;
            lastWrappedYawRad = wrappedYawRad;
            hasYawSample = true;
            return;
        }

        accumulatedYawRad += MathUtil.angleModulus(wrappedYawRad - lastWrappedYawRad);
        lastWrappedYawRad = wrappedYawRad;
    }

}
