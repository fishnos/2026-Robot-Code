package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.configs.VisionConfig;
import org.junit.jupiter.api.Test;

class VisionUtilTest {
    @Test
    void fieldDimensionsStayAlignedWithBLineFlippingUtil() {
        assertTrue(VisionUtil.fieldDimensionsMatchFlippingUtil());
    }

    @Test
    void isPoseWithinField_acceptsInBoundsPose() {
        assertTrue(
            VisionUtil.isPoseWithinField(
                new Pose3d(
                    VisionUtil.getFieldLengthMeters() / 2.0,
                    VisionUtil.getFieldWidthMeters() / 2.0,
                    0.0,
                    new Rotation3d()
                )
            )
        );
    }

    @Test
    void isPoseWithinField_rejectsOutOfBoundsPose() {
        assertFalse(VisionUtil.isPoseWithinField(new Pose3d(-0.01, 2.0, 0.0, new Rotation3d())));
        assertFalse(
            VisionUtil.isPoseWithinField(
                new Pose3d(VisionUtil.getFieldLengthMeters() + 0.01, 2.0, 0.0, new Rotation3d())
            )
        );
        assertFalse(
            VisionUtil.isPoseWithinField(
                new Pose3d(2.0, VisionUtil.getFieldWidthMeters() + 0.01, 0.0, new Rotation3d())
            )
        );
    }

    @Test
    void cameraConfigDefaultsObservationModeToBoth() {
        VisionConfig.CameraConfig cameraConfig = new VisionConfig.CameraConfig();

        assertTrue(cameraConfig.getObservationMode() == VisionConfig.ObservationMode.BOTH);
    }

    @Test
    void observationModeBothAllowsAllVisionTypes() {
        assertTrue(
            VisionUtil.isObservationTypeAllowed(
                VisionIO.PoseObservationType.MEGATAG_1,
                VisionConfig.ObservationMode.BOTH,
                false
            )
        );
        assertTrue(
            VisionUtil.isObservationTypeAllowed(
                VisionIO.PoseObservationType.MEGATAG_2,
                VisionConfig.ObservationMode.BOTH,
                false
            )
        );
        assertTrue(
            VisionUtil.isObservationTypeAllowed(
                VisionIO.PoseObservationType.PHOTONVISION,
                VisionConfig.ObservationMode.BOTH,
                false
            )
        );
    }

    @Test
    void observationModeMt1OnlyRejectsMegatag2() {
        assertTrue(
            VisionUtil.isObservationTypeAllowed(
                VisionIO.PoseObservationType.MEGATAG_1,
                VisionConfig.ObservationMode.MT1_ONLY,
                false
            )
        );
        assertFalse(
            VisionUtil.isObservationTypeAllowed(
                VisionIO.PoseObservationType.MEGATAG_2,
                VisionConfig.ObservationMode.MT1_ONLY,
                false
            )
        );
        assertTrue(
            VisionUtil.isObservationTypeAllowed(
                VisionIO.PoseObservationType.PHOTONVISION,
                VisionConfig.ObservationMode.MT1_ONLY,
                false
            )
        );
    }

    @Test
    void observationModeMt2OnlyRejectsMegatag1() {
        assertFalse(
            VisionUtil.isObservationTypeAllowed(
                VisionIO.PoseObservationType.MEGATAG_1,
                VisionConfig.ObservationMode.MT2_ONLY,
                false
            )
        );
        assertTrue(
            VisionUtil.isObservationTypeAllowed(
                VisionIO.PoseObservationType.MEGATAG_2,
                VisionConfig.ObservationMode.MT2_ONLY,
                false
            )
        );
        assertTrue(
            VisionUtil.isObservationTypeAllowed(
                VisionIO.PoseObservationType.PHOTONVISION,
                VisionConfig.ObservationMode.MT2_ONLY,
                false
            )
        );
    }

    @Test
    void hybridObservationModeUsesMt2WhenEnabled() {
        assertFalse(
            VisionUtil.isObservationTypeAllowed(
                VisionIO.PoseObservationType.MEGATAG_1,
                VisionConfig.ObservationMode.MT2_WHILE_ENABLED_MT1_WHILE_DISABLED,
                false
            )
        );
        assertTrue(
            VisionUtil.isObservationTypeAllowed(
                VisionIO.PoseObservationType.MEGATAG_2,
                VisionConfig.ObservationMode.MT2_WHILE_ENABLED_MT1_WHILE_DISABLED,
                false
            )
        );
        assertTrue(
            VisionUtil.isObservationTypeAllowed(
                VisionIO.PoseObservationType.PHOTONVISION,
                VisionConfig.ObservationMode.MT2_WHILE_ENABLED_MT1_WHILE_DISABLED,
                false
            )
        );
    }

    @Test
    void hybridObservationModeUsesMt1WhenDisabled() {
        assertTrue(
            VisionUtil.isObservationTypeAllowed(
                VisionIO.PoseObservationType.MEGATAG_1,
                VisionConfig.ObservationMode.MT2_WHILE_ENABLED_MT1_WHILE_DISABLED,
                true
            )
        );
        assertFalse(
            VisionUtil.isObservationTypeAllowed(
                VisionIO.PoseObservationType.MEGATAG_2,
                VisionConfig.ObservationMode.MT2_WHILE_ENABLED_MT1_WHILE_DISABLED,
                true
            )
        );
        assertTrue(
            VisionUtil.isObservationTypeAllowed(
                VisionIO.PoseObservationType.PHOTONVISION,
                VisionConfig.ObservationMode.MT2_WHILE_ENABLED_MT1_WHILE_DISABLED,
                true
            )
        );
    }
}
