package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.lib.util.LimelightHelpers;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
    private static final double CONNECTION_TIMEOUT_MILLISECONDS = 250.0;

    private final String name;
    private final Supplier<Rotation3d> rotationSupplier;
    private final NetworkTableEntry heartbeatEntry;
    private final DoubleSubscriber latencySubscriber;
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;
    private final DoubleArraySubscriber megatag1Subscriber;
    private final DoubleArraySubscriber megatag2Subscriber;

    /**
     * Creates a new VisionIOLimelight.
     *
     * @param name The configured name of the Limelight.
     * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
     */
    public VisionIOLimelight(String name, Supplier<Rotation3d> rotationSupplier) {
        var table = NetworkTableInstance.getDefault().getTable(name);
        this.name = name;
        this.rotationSupplier = rotationSupplier;
        heartbeatEntry = table.getEntry("hb");
        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
        tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
        megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        megatag2Subscriber =
            table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    }

    @Override
    public boolean publishRobotOrientation() {
        Rotation3d orientation = rotationSupplier.get();
        LimelightHelpers.SetRobotOrientation_NoFlush(
            name,
            Math.toDegrees(orientation.getZ()),
            0.0,
            Math.toDegrees(orientation.getY()),
            0.0,
            Math.toDegrees(orientation.getX()),
            0.0
        );
        return true;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        long lastUpdateMicros = getLatestChangeMicros(
            heartbeatEntry.getLastChange(),
            latencySubscriber.getLastChange(),
            txSubscriber.getLastChange(),
            tySubscriber.getLastChange(),
            megatag1Subscriber.getLastChange(),
            megatag2Subscriber.getLastChange()
        );
        inputs.connected =
            ((RobotController.getFPGATime() - lastUpdateMicros) / 1000.0) < CONNECTION_TIMEOUT_MILLISECONDS;

        // Update target observation
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

        // Read new pose observations from NetworkTables
        Set<Integer> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new ArrayList<>();
        int rawMegatag1ObservationCount = 0;
        int rawMegatag2ObservationCount = 0;
        for (TimestampedDoubleArray rawSample : megatag1Subscriber.readQueue()) {
            collectTagIds(tagIds, rawSample.value);
            Optional<PoseObservation> poseObservation =
                parsePoseObservation(rawSample, PoseObservationType.MEGATAG_1);
            if (poseObservation.isPresent()) {
                poseObservations.add(poseObservation.get());
                rawMegatag1ObservationCount++;
            }
        }
        for (TimestampedDoubleArray rawSample : megatag2Subscriber.readQueue()) {
            collectTagIds(tagIds, rawSample.value);
            Optional<PoseObservation> poseObservation =
                parsePoseObservation(rawSample, PoseObservationType.MEGATAG_2);
            if (poseObservation.isPresent()) {
                poseObservations.add(poseObservation.get());
                rawMegatag2ObservationCount++;
            }
        }
        inputs.robotPoseObservations = poseObservations.toArray(new PoseObservation[0]);
        inputs.rawMegatag1ObservationCount = rawMegatag1ObservationCount;
        inputs.rawMegatag2ObservationCount = rawMegatag2ObservationCount;
        inputs.rawObservationCount = poseObservations.size();

        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }

        // Limelight bounds filtering only needs field dimensions, not an AprilTag layout load.
        inputs.tagPoses = new Pose3d[0];
    }

    /** Parses the 3D pose from a Limelight botpose array. */
    private static Pose3d parsePose(double[] rawLLArray) {
        return new Pose3d(
            rawLLArray[0],
            rawLLArray[1],
            rawLLArray[2],
            new Rotation3d(
                Units.degreesToRadians(rawLLArray[3]),
                Units.degreesToRadians(rawLLArray[4]),
                Units.degreesToRadians(rawLLArray[5])));
    }

    private static Optional<PoseObservation> parsePoseObservation(
        TimestampedDoubleArray rawSample,
        PoseObservationType type
    ) {
        double[] values = rawSample.value;
        if (values.length < 11) {
            return Optional.empty();
        }

        int tagCount = (int) values[7];
        if (tagCount < 0) {
            return Optional.empty();
        }

        double timestampSeconds = rawSample.timestamp * 1.0e-6 - values[6] * 1.0e-3;
        double averageTagDistance = values[9];
        if (!Double.isFinite(timestampSeconds)
            || !Double.isFinite(averageTagDistance)
            || !Double.isFinite(values[0])
            || !Double.isFinite(values[1])
            || !Double.isFinite(values[2])) {
            return Optional.empty();
        }

        double ambiguity =
            type == PoseObservationType.MEGATAG_1 && values.length >= 18 ? values[17] : 0.0;
        return Optional.of(
            new PoseObservation(
                timestampSeconds,
                parsePose(values),
                ambiguity,
                tagCount,
                averageTagDistance,
                type
            )
        );
    }

    private static void collectTagIds(Set<Integer> tagIds, double[] rawSample) {
        if (rawSample.length < 11) {
            return;
        }

        int tagCount = (int) rawSample[7];
        if (tagCount <= 0) {
            return;
        }

        int expectedValueCount = 11 + (tagCount * 7);
        if (rawSample.length < expectedValueCount) {
            return;
        }

        for (int i = 11; i < expectedValueCount; i += 7) {
            tagIds.add((int) rawSample[i]);
        }
    }

    private static long getLatestChangeMicros(long... timestamps) {
        long latestChangeMicros = 0;
        for (long timestamp : timestamps) {
            latestChangeMicros = Math.max(latestChangeMicros, timestamp);
        }
        return latestChangeMicros;
    }
}
