package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;
import frc.robot.configs.VisionConfig;
import frc.robot.constants.Constants;
import frc.robot.constants.vision.VisionConstants;
import frc.robot.lib.util.ConfigLoader;
import frc.robot.lib.util.LimelightHelpers;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private static Vision instance = null;
    private static final VisionConfig config = ConfigLoader.load(
        "vision",
        ConfigLoader.getModeFolder(Constants.SimOnlySubsystems.VISION),
        VisionConfig.class
    );
    public static Vision getInstance() {
        if (instance == null) {
            RobotState robotState = RobotState.getInstance();
            boolean useSimulation = Constants.shouldUseSimulation(Constants.SimOnlySubsystems.VISION);
            List<VisionIO> ioList = new ArrayList<>();
            List<VisionConfig.CameraConfig> cameraConfigs =
                config.cameras != null ? config.cameras : List.of();
            if (useSimulation) {
                for (VisionConfig.CameraConfig camera : cameraConfigs) {
                    ioList.add(
                        new VisionIOPhotonVisionSim(
                            camera.name,
                            camera.getRobotToCamera(),
                            () -> robotState.getEstimatedPose()
                        )
                    );
                }
            } else {
                for (VisionConfig.CameraConfig camera : cameraConfigs) {
                    ioList.add(new VisionIOLimelight(camera.name, robotState::getLastOrientation));
                }
            }
            instance = new Vision(
                robotState::addVisionObservation,
                () -> Rotation2d.fromRadians(robotState.getFieldRelativeSpeeds().omegaRadiansPerSecond),
                ioList.toArray(new VisionIO[0])
            );

        }
        return instance;
    }
    
    private final Consumer<VisionObservation>  consumer;
    private final Supplier<Rotation2d> rotationRateSupplier;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;
    private final TimeInterpolatableBuffer<Rotation2d> rotationRateBuffer;
    private final String[] limelightNames;

    private Vision(Consumer<VisionObservation> consumer, Supplier<Rotation2d> rotationRateSupplier, VisionIO... io) {
        this.consumer = consumer;
        this.rotationRateSupplier = rotationRateSupplier;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] =
                new Alert(
                    "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }

        // Initialize rotation rate buffer (keep 2 seconds of data)
        this.rotationRateBuffer = TimeInterpolatableBuffer.createBuffer(2.0);

        List<String> limelightNameList = new ArrayList<>();
        if (config.cameras != null) {
            for (VisionConfig.CameraConfig camera : config.cameras) {
                if (camera.name != null
                    && !camera.name.isBlank()
                    && !limelightNameList.contains(camera.name)) {
                    limelightNameList.add(camera.name);
                }
            }
        }
        this.limelightNames = limelightNameList.toArray(new String[0]);
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    @Override
    public void periodic() {
        int imuMode = DriverStation.isDisabled() ? config.disabledImuMode : config.enabledImuMode;
        for (String cameraName : limelightNames) {
            LimelightHelpers.SetIMUMode(cameraName, imuMode);
        }

        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        // Record current rotation rate
        double currentTime = Timer.getTimestamp();

        // Store rotation rate in buffer
        rotationRateBuffer.addSample(currentTime, rotationRateSupplier.get());

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = VisionConstants.aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].robotPoseObservations) {
                // Check rotation rate at observation timestamp
                boolean rotationRateTooHigh = false;
                var rotationRateAtTime = rotationRateBuffer.getSample(observation.timestamp());
                if (rotationRateAtTime.isPresent()) {
                    double observationRotationRateDegreesPerSecond = Math.abs(rotationRateAtTime.get().getDegrees());
                    
                    // Use different thresholds based on observation type
                    double maxRotationRateThreshold;
                    if (observation.type() == PoseObservationType.MEGATAG_1) {
                        maxRotationRateThreshold = config.maxRotationRateMegatag1DegreesPerSecond;
                    } else if (observation.type() == PoseObservationType.MEGATAG_2) {
                        maxRotationRateThreshold = config.maxRotationRateMegatag2DegreesPerSecond;
                    } else {
                        // Default to MegaTag1 threshold for other types (like PhotonVision)
                        maxRotationRateThreshold = config.maxRotationRateMegatag1DegreesPerSecond;
                    }
                    
                    rotationRateTooHigh = observationRotationRateDegreesPerSecond > maxRotationRateThreshold;
                }
                
                // Check whether to reject pose
                boolean rejectPose =
                    observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                            && observation.ambiguity() > config.maxAmbiguity) // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ())
                            > config.maxZError // Must have realistic Z coordinate
                        || rotationRateTooHigh // Must not be rotating too fast

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > VisionConstants.aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > VisionConstants.aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor =
                    Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = config.linearStdDevBaseline * stdDevFactor;
                double angularStdDev = config.angularStdDevBaseline * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_1) {
                    linearStdDev *= config.linearStdDevMegatag1Factor;
                    angularStdDev *= config.angularStdDevMegatag1Factor;
                } else if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= config.linearStdDevMegatag2Factor;
                    angularStdDev *= config.angularStdDevMegatag2Factor;
                }
                if (config.cameraStdDevFactors != null && cameraIndex < config.cameraStdDevFactors.length) {
                    linearStdDev *= config.cameraStdDevFactors[cameraIndex];
                    angularStdDev *= config.cameraStdDevFactors[cameraIndex];
                }

                if (!Double.isFinite(linearStdDev) || !Double.isFinite(angularStdDev)) {
                    robotPosesRejected.add(observation.pose());
                    continue;
                }

                // Send vision observation
                consumer.accept(
                    new VisionObservation(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                    )
                );
            }

            // Log camera datadata
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput(
            "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput(
            "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
            "Vision/Summary/RobotPosesAccepted",
            allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
            "Vision/Summary/RobotPosesRejected",
            allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }
}