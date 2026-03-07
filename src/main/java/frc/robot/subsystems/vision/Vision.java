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
import frc.robot.lib.util.ConfigLoader;
import frc.robot.lib.util.LimelightHelpers;
import frc.robot.lib.util.LoopCycleProfiler;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private static final double DETAILED_POSE_LOG_PERIOD_SECONDS = 0.1;
    private static final double IMU_MODE_REASSERT_PERIOD_SECONDS = 1.0;
    private static final boolean ENABLE_DETAILED_POSE_LOGGING =
        Constants.currentMode == Constants.Mode.REPLAY || Constants.VERBOSE_LOGGING_ENABLED;

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
                    ioList.add(new VisionIO() {});
                }
            } else {
                for (VisionConfig.CameraConfig camera : cameraConfigs) {
                    ioList.add(new VisionIOLimelight(camera.name, robotState::getLastOrientation));
                }
            }
            instance = new Vision(
                robotState::addVisionObservation,
                () -> Math.toDegrees(robotState.getRobotRelativeSpeeds().omegaRadiansPerSecond),
                ioList.toArray(new VisionIO[0])
            );

        }
        return instance;
    }
    
    private final Consumer<VisionObservation> consumer;
    private final DoubleSupplier rotationRateDegreesPerSecondSupplier;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;
    private final TimeInterpolatableBuffer<Double> rotationRateBuffer;
    private final String[] limelightNames;
    private final boolean[] lastConnectedStates;
    private int lastImuMode = Integer.MIN_VALUE;
    private double lastImuModeWriteTimestampSeconds = Double.NEGATIVE_INFINITY;
    private double lastDetailedPoseLogTimestampSeconds = Double.NEGATIVE_INFINITY;

    private Vision(
        Consumer<VisionObservation> consumer,
        DoubleSupplier rotationRateDegreesPerSecondSupplier,
        VisionIO... io
    ) {
        this.consumer = consumer;
        this.rotationRateDegreesPerSecondSupplier = rotationRateDegreesPerSecondSupplier;
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
        this.rotationRateBuffer = TimeInterpolatableBuffer.createDoubleBuffer(2.0);
        this.lastConnectedStates = new boolean[io.length];

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

        Logger.recordOutput("Vision/FieldLengthMeters", VisionUtil.getFieldLengthMeters());
        Logger.recordOutput("Vision/FieldWidthMeters", VisionUtil.getFieldWidthMeters());
        boolean fieldDimensionsMatchBLine = VisionUtil.fieldDimensionsMatchFlippingUtil();
        Logger.recordOutput("Vision/FieldDimensionsMatchBLine", fieldDimensionsMatchBLine);
        if (!fieldDimensionsMatchBLine) {
            DriverStation.reportWarning(VisionUtil.getFieldDimensionMismatchSummary(), false);
        }
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
        long periodicStartNanos = LoopCycleProfiler.markStart();

        long imuModeStartNanos = LoopCycleProfiler.markStart();
        double currentTime = Timer.getTimestamp();
        int imuMode = DriverStation.isDisabled() ? config.disabledImuMode : config.enabledImuMode;
        boolean wroteImuModeThisCycle = false;
        if (imuMode != lastImuMode
            || currentTime - lastImuModeWriteTimestampSeconds >= IMU_MODE_REASSERT_PERIOD_SECONDS) {
            writeImuMode(imuMode, currentTime);
            wroteImuModeThisCycle = true;
        }
        LoopCycleProfiler.endSection("Vision/SetIMUMode", imuModeStartNanos);

        long orientationPublishStartNanos = LoopCycleProfiler.markStart();
        boolean shouldFlushRobotOrientation = false;
        for (VisionIO ioImplementation : io) {
            shouldFlushRobotOrientation |= ioImplementation.publishRobotOrientation();
        }
        if (shouldFlushRobotOrientation) {
            LimelightHelpers.Flush();
        }
        LoopCycleProfiler.endSection("Vision/PublishRobotOrientation", orientationPublishStartNanos);

        long inputUpdateStartNanos = LoopCycleProfiler.markStart();
        boolean sawReconnectThisCycle = false;
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            if (inputs[i].connected && !lastConnectedStates[i]) {
                sawReconnectThisCycle = true;
            }
            lastConnectedStates[i] = inputs[i].connected;
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }
        if (sawReconnectThisCycle && !wroteImuModeThisCycle) {
            writeImuMode(imuMode, currentTime);
        }
        LoopCycleProfiler.endSection("Vision/UpdateAndProcessInputs", inputUpdateStartNanos);

        // Record current rotation rate
        long rotationRateStartNanos = LoopCycleProfiler.markStart();

        // Store rotation rate in buffer
        rotationRateBuffer.addSample(currentTime, rotationRateDegreesPerSecondSupplier.getAsDouble());
        LoopCycleProfiler.endSection("Vision/RotationRateBufferUpdate", rotationRateStartNanos);

        // Initialize logging values
        boolean shouldLogDetailedPoseArrays =
            ENABLE_DETAILED_POSE_LOGGING
                && currentTime - lastDetailedPoseLogTimestampSeconds >= DETAILED_POSE_LOG_PERIOD_SECONDS;
        if (shouldLogDetailedPoseArrays) {
            lastDetailedPoseLogTimestampSeconds = currentTime;
        }

        long cameraProcessingStartNanos = LoopCycleProfiler.markStart();
        int totalAcceptedCount = 0;
        int totalRejectedCount = 0;
        int totalObservationCount = 0;
        int totalRawMegatag1Count = 0;
        int totalRawMegatag2Count = 0;
        int totalRawObservationCount = 0;
        int totalCoalescedObservationCount = 0;
        int totalCoalescedDropCount = 0;

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            long perCameraStartNanos = LoopCycleProfiler.markStart();
            String cameraTimingPrefix = "Vision/Camera" + Integer.toString(cameraIndex);

            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            int poseObservationCount = inputs[cameraIndex].robotPoseObservations.length;
            List<Pose3d> robotPoses = shouldLogDetailedPoseArrays ? new ArrayList<>(poseObservationCount) : null;
            List<Pose3d> robotPosesAccepted = shouldLogDetailedPoseArrays ? new ArrayList<>(poseObservationCount) : null;
            List<Pose3d> robotPosesRejected = shouldLogDetailedPoseArrays ? new ArrayList<>(poseObservationCount) : null;
            List<String> poseTypes = shouldLogDetailedPoseArrays ? new ArrayList<>(poseObservationCount) : null;
            List<String> poseTypesAccepted = shouldLogDetailedPoseArrays ? new ArrayList<>(poseObservationCount) : null;
            List<String> poseTypesRejected = shouldLogDetailedPoseArrays ? new ArrayList<>(poseObservationCount) : null;
            int acceptedCount = 0;
            int rejectedCount = 0;
            totalRawMegatag1Count += inputs[cameraIndex].rawMegatag1ObservationCount;
            totalRawMegatag2Count += inputs[cameraIndex].rawMegatag2ObservationCount;
            totalRawObservationCount += inputs[cameraIndex].rawObservationCount;
            totalCoalescedObservationCount += inputs[cameraIndex].coalescedObservationCount;
            totalCoalescedDropCount += inputs[cameraIndex].coalescedDropCount;

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].robotPoseObservations) {
                totalObservationCount++;
                boolean translationRateTooHigh = false;
                boolean rotationRateTooHigh = false;
                var rotationRateAtTime = rotationRateBuffer.getSample(observation.timestamp());
                if (rotationRateAtTime.isPresent()) {
                    double observationRotationRateDegreesPerSecond = Math.abs(rotationRateAtTime.get());
                    translationRateTooHigh =
                        observationRotationRateDegreesPerSecond
                            > getMaxRotationRateThreshold(observation.type());
                    rotationRateTooHigh =
                        observationRotationRateDegreesPerSecond
                            > getMaxRotationRateThreshold(observation.rotationType());
                }

                boolean rejectPose =
                    observation.tagCount() == 0
                        || (observation.tagCount() == 1
                            && observation.ambiguity() > config.maxAmbiguity)
                        || Math.abs(observation.pose().getZ()) > config.maxZError
                        || !VisionUtil.isPoseWithinField(observation.pose())
                        || translationRateTooHigh;

                if (shouldLogDetailedPoseArrays) {
                    robotPoses.add(observation.pose());
                    poseTypes.add(observation.type().name());
                }
                if (rejectPose) {
                    rejectedCount++;
                    totalRejectedCount++;
                    if (shouldLogDetailedPoseArrays) {
                        robotPosesRejected.add(observation.pose());
                        poseTypesRejected.add(observation.type().name());
                    }
                }

                if (rejectPose) {
                    continue;
                }

                double linearStdDev =
                    config.linearStdDevBaseline
                        * computeStdDevFactor(observation.averageTagDistance(), observation.tagCount())
                        * getLinearStdDevTypeFactor(observation.type());
                boolean useMegatag1Rotation =
                    observation.rotationType() == PoseObservationType.MEGATAG_1
                        && observation.rotationTagCount() > 0
                        && !(observation.rotationTagCount() == 1
                            && observation.rotationAmbiguity() > config.maxAmbiguity)
                        && !rotationRateTooHigh;
                double angularSourceDistance =
                    useMegatag1Rotation
                        ? observation.rotationAverageTagDistance()
                        : observation.averageTagDistance();
                int angularSourceTagCount =
                    useMegatag1Rotation ? observation.rotationTagCount() : observation.tagCount();
                PoseObservationType angularSourceType =
                    useMegatag1Rotation ? observation.rotationType() : PoseObservationType.MEGATAG_2;
                double angularStdDev =
                    config.angularStdDevBaseline
                        * computeStdDevFactor(angularSourceDistance, angularSourceTagCount)
                        * getAngularStdDevTypeFactor(angularSourceType);
                if (config.cameraStdDevFactors != null && cameraIndex < config.cameraStdDevFactors.length) {
                    linearStdDev *= config.cameraStdDevFactors[cameraIndex];
                    angularStdDev *= config.cameraStdDevFactors[cameraIndex];
                }

                if (!Double.isFinite(linearStdDev) || !Double.isFinite(angularStdDev)) {
                    rejectedCount++;
                    totalRejectedCount++;
                    if (shouldLogDetailedPoseArrays) {
                        robotPosesRejected.add(observation.pose());
                    }
                    continue;
                }

                acceptedCount++;
                totalAcceptedCount++;
                if (shouldLogDetailedPoseArrays) {
                    robotPosesAccepted.add(observation.pose());
                    poseTypesAccepted.add(observation.type().name());
                }

                consumer.accept(
                    new VisionObservation(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                    )
                );
            }

            Logger.recordOutput(cameraTimingPrefix + "/PoseObservationCount", poseObservationCount);
            Logger.recordOutput(cameraTimingPrefix + "/PoseAcceptedCount", acceptedCount);
            Logger.recordOutput(cameraTimingPrefix + "/PoseRejectedCount", rejectedCount);
            Logger.recordOutput(
                cameraTimingPrefix + "/RawMegatag1ObservationCount",
                inputs[cameraIndex].rawMegatag1ObservationCount
            );
            Logger.recordOutput(
                cameraTimingPrefix + "/RawMegatag2ObservationCount",
                inputs[cameraIndex].rawMegatag2ObservationCount
            );
            Logger.recordOutput(
                cameraTimingPrefix + "/RawObservationCount",
                inputs[cameraIndex].rawObservationCount
            );
            Logger.recordOutput(
                cameraTimingPrefix + "/CoalescedObservationCount",
                inputs[cameraIndex].coalescedObservationCount
            );
            Logger.recordOutput(
                cameraTimingPrefix + "/CoalescedDropCount",
                inputs[cameraIndex].coalescedDropCount
            );
            Logger.recordOutput(
                cameraTimingPrefix + "/CoalescedGroupSizes",
                inputs[cameraIndex].coalescedGroupSizes
            );
            Logger.recordOutput(
                cameraTimingPrefix + "/CoalescedTranslationSourceTypes",
                inputs[cameraIndex].coalescedTranslationSourceTypes
            );
            Logger.recordOutput(
                cameraTimingPrefix + "/CoalescedTranslationDecisionReasons",
                inputs[cameraIndex].coalescedTranslationDecisionReasons
            );
            Logger.recordOutput(
                cameraTimingPrefix + "/CoalescedRotationSourceTypes",
                inputs[cameraIndex].coalescedRotationSourceTypes
            );
            Logger.recordOutput(
                cameraTimingPrefix + "/CoalescedRotationDecisionReasons",
                inputs[cameraIndex].coalescedRotationDecisionReasons
            );
            if (shouldLogDetailedPoseArrays) {
                Logger.recordOutput(cameraTimingPrefix + "/TagPoses", inputs[cameraIndex].tagPoses);
                Logger.recordOutput(
                    cameraTimingPrefix + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
                Logger.recordOutput(
                    cameraTimingPrefix + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
                Logger.recordOutput(
                    cameraTimingPrefix + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
                Logger.recordOutput(
                    cameraTimingPrefix + "/PoseTypes",
                    poseTypes.toArray(new String[poseTypes.size()]));
                Logger.recordOutput(
                    cameraTimingPrefix + "/PoseTypesAccepted",
                    poseTypesAccepted.toArray(new String[poseTypesAccepted.size()]));
                Logger.recordOutput(
                    cameraTimingPrefix + "/PoseTypesRejected",
                    poseTypesRejected.toArray(new String[poseTypesRejected.size()]));
            }

            LoopCycleProfiler.endSection(cameraTimingPrefix + "/Total", perCameraStartNanos);
        }
        LoopCycleProfiler.endSection("Vision/CameraProcessing", cameraProcessingStartNanos);

        long summaryLogStartNanos = LoopCycleProfiler.markStart();
        Logger.recordOutput("Vision/Summary/PoseObservationCount", totalObservationCount);
        Logger.recordOutput("Vision/Summary/PoseAcceptedCount", totalAcceptedCount);
        Logger.recordOutput("Vision/Summary/PoseRejectedCount", totalRejectedCount);
        Logger.recordOutput("Vision/Summary/RawMegatag1ObservationCount", totalRawMegatag1Count);
        Logger.recordOutput("Vision/Summary/RawMegatag2ObservationCount", totalRawMegatag2Count);
        Logger.recordOutput("Vision/Summary/RawObservationCount", totalRawObservationCount);
        Logger.recordOutput(
            "Vision/Summary/CoalescedObservationCount",
            totalCoalescedObservationCount
        );
        Logger.recordOutput("Vision/Summary/CoalescedDropCount", totalCoalescedDropCount);
        LoopCycleProfiler.endSection("Vision/SummaryLogging", summaryLogStartNanos);

        LoopCycleProfiler.endSection("Vision/PeriodicTotal", periodicStartNanos);
    }

    private void writeImuMode(int imuMode, double currentTime) {
        for (String cameraName : limelightNames) {
            LimelightHelpers.SetIMUMode(cameraName, imuMode);
        }
        lastImuMode = imuMode;
        lastImuModeWriteTimestampSeconds = currentTime;
    }

    private double getMaxRotationRateThreshold(PoseObservationType type) {
        return switch (type) {
            case MEGATAG_1, PHOTONVISION -> config.maxRotationRateMegatag1DegreesPerSecond;
            case MEGATAG_2 -> config.maxRotationRateMegatag2DegreesPerSecond;
        };
    }

    private double getLinearStdDevTypeFactor(PoseObservationType type) {
        return switch (type) {
            case MEGATAG_1, PHOTONVISION -> config.linearStdDevMegatag1Factor;
            case MEGATAG_2 -> config.linearStdDevMegatag2Factor;
        };
    }

    private double getAngularStdDevTypeFactor(PoseObservationType type) {
        return switch (type) {
            case MEGATAG_1, PHOTONVISION -> config.angularStdDevMegatag1Factor;
            case MEGATAG_2 -> config.angularStdDevMegatag2Factor;
        };
    }

    private static double computeStdDevFactor(double averageTagDistance, int tagCount) {
        return Math.pow(averageTagDistance, 2.0) / Math.max(tagCount, 1);
    }
}
