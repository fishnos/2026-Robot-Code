package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.configs.VisionConfig.ObservationMode;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

final class VisionUtil {
    static final double FIELD_DIMENSION_MATCH_TOLERANCE_METERS = 0.005;

    private VisionUtil() {}

    static double getFieldLengthMeters() {
        return FieldConstants.fieldLength;
    }

    static double getFieldWidthMeters() {
        return FieldConstants.fieldWidth;
    }

    static boolean isPoseWithinField(Pose3d pose) {
        return pose.getX() >= 0.0
            && pose.getX() <= getFieldLengthMeters()
            && pose.getY() >= 0.0
            && pose.getY() <= getFieldWidthMeters();
    }

    static boolean fieldDimensionsMatchFlippingUtil() {
        return Math.abs(getFieldLengthMeters() - FlippingUtil.fieldSizeX)
                <= FIELD_DIMENSION_MATCH_TOLERANCE_METERS
            && Math.abs(getFieldWidthMeters() - FlippingUtil.fieldSizeY)
                <= FIELD_DIMENSION_MATCH_TOLERANCE_METERS;
    }

    static String getFieldDimensionMismatchSummary() {
        return String.format(
            "Vision field dimensions mismatch B-Line FlippingUtil. FieldConstants=(%.6f, %.6f) "
                + "FlippingUtil=(%.6f, %.6f)",
            getFieldLengthMeters(),
            getFieldWidthMeters(),
            FlippingUtil.fieldSizeX,
            FlippingUtil.fieldSizeY
        );
    }

    static boolean isObservationTypeAllowed(
        PoseObservationType observationType,
        ObservationMode observationMode,
        boolean isDisabled
    ) {
        ObservationMode effectiveObservationMode =
            getEffectiveObservationMode(observationMode, isDisabled);
        if (observationType == PoseObservationType.PHOTONVISION) {
            return true;
        }

        return switch (effectiveObservationMode) {
            case BOTH -> true;
            case MT1_ONLY -> observationType == PoseObservationType.MEGATAG_1;
            case MT2_ONLY -> observationType == PoseObservationType.MEGATAG_2;
            case MT2_WHILE_ENABLED_MT1_WHILE_DISABLED -> false;
        };
    }

    static ObservationMode getEffectiveObservationMode(
        ObservationMode observationMode,
        boolean isDisabled
    ) {
        ObservationMode configuredObservationMode =
            observationMode != null ? observationMode : ObservationMode.BOTH;

        return switch (configuredObservationMode) {
            case MT2_WHILE_ENABLED_MT1_WHILE_DISABLED ->
                isDisabled ? ObservationMode.MT1_ONLY : ObservationMode.MT2_ONLY;
            case BOTH, MT1_ONLY, MT2_ONLY -> configuredObservationMode;
        };
    }
}
