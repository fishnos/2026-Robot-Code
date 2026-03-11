package frc.robot.configs;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;

public class VisionConfig {
    public static enum ObservationMode {
        BOTH,
        MT1_ONLY,
        MT2_ONLY,
        MT2_WHILE_ENABLED_MT1_WHILE_DISABLED
    }

    public static class CameraConfig {
        public String name;
        public double x;
        public double y;
        public double z;
        public double rollDeg;
        public double pitchDeg;
        public double yawDeg;
        public ObservationMode observationMode;

        public Transform3d getRobotToCamera() {
            return new Transform3d(
                x,
                y,
                z,
                new Rotation3d(
                    Math.toRadians(rollDeg),
                    Math.toRadians(pitchDeg),
                    Math.toRadians(yawDeg)));
        }

        public ObservationMode getObservationMode() {
            return observationMode != null ? observationMode : ObservationMode.BOTH;
        }
    }

    public List<CameraConfig> cameras;

    public int disabledImuMode;
    public int enabledImuMode;

    public double maxAmbiguity;
    public double maxZError;

    public double linearStdDevBaseline;
    public double angularStdDevBaseline;
    public double linearStdDevMegatag1Factor;
    public double angularStdDevMegatag1Factor;
    public double linearStdDevMegatag2Factor;
    public double angularStdDevMegatag2Factor;

    public double maxRotationRateMegatag1DegreesPerSecond;
    public double maxRotationRateMegatag2DegreesPerSecond;

    public double[] cameraStdDevFactors;
}
