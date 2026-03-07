package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.List;
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
    void coalesceCorrelatedObservations_prefersMegaTag2ForSameFrame() {
        VisionIO.PoseObservation megatag1Observation =
            new VisionIO.PoseObservation(
                1.000,
                new Pose3d(1.0, 2.0, 0.0, new Rotation3d(0.0, 0.0, 0.75)),
                0.3,
                2,
                3.0,
                VisionIO.PoseObservationType.MEGATAG_1
            );
        VisionIO.PoseObservation megatag2Observation =
            new VisionIO.PoseObservation(
                1.003,
                new Pose3d(1.1, 2.1, 0.0, new Rotation3d(0.0, 0.0, 0.25)),
                0.0,
                2,
                3.0,
                VisionIO.PoseObservationType.MEGATAG_2
            );

        VisionUtil.CoalescedObservationsResult observations =
            VisionUtil.coalesceCorrelatedObservations(List.of(megatag1Observation, megatag2Observation));

        assertEquals(2, observations.rawObservationCount());
        assertEquals(1, observations.observations().size());
        assertEquals(1, observations.coalescedObservationCount());
        assertEquals(VisionIO.PoseObservationType.MEGATAG_2, observations.observations().get(0).type());
        assertEquals(VisionIO.PoseObservationType.MEGATAG_1, observations.observations().get(0).rotationType());
        assertEquals(1.1, observations.observations().get(0).pose().getX(), 1e-9);
        assertEquals(2.1, observations.observations().get(0).pose().getY(), 1e-9);
        assertEquals(0.75, observations.observations().get(0).pose().getRotation().getZ(), 1e-9);
        assertEquals(1, observations.coalescedDropCount());
        assertEquals(2, observations.groupSizes()[0]);
        assertEquals("MEGATAG_2", observations.translationSourceTypes()[0]);
        assertEquals("preferred_type", observations.translationDecisionReasons()[0]);
        assertEquals("MEGATAG_1", observations.rotationSourceTypes()[0]);
        assertEquals("single_megatag1_rotation", observations.rotationDecisionReasons()[0]);
    }

    @Test
    void coalesceCorrelatedObservations_keepsDistinctFrames() {
        VisionIO.PoseObservation firstObservation =
            new VisionIO.PoseObservation(
                1.000,
                new Pose3d(1.0, 2.0, 0.0, new Rotation3d()),
                0.1,
                1,
                3.0,
                VisionIO.PoseObservationType.MEGATAG_1
            );
        VisionIO.PoseObservation secondObservation =
            new VisionIO.PoseObservation(
                1.020,
                new Pose3d(1.1, 2.1, 0.0, new Rotation3d()),
                0.1,
                1,
                3.0,
                VisionIO.PoseObservationType.MEGATAG_2
            );

        VisionUtil.CoalescedObservationsResult observations =
            VisionUtil.coalesceCorrelatedObservations(List.of(firstObservation, secondObservation));

        assertEquals(2, observations.rawObservationCount());
        assertEquals(2, observations.observations().size());
        assertEquals(2, observations.coalescedObservationCount());
        assertEquals(0, observations.coalescedDropCount());
        assertEquals(1, observations.groupSizes()[0]);
        assertEquals(1, observations.groupSizes()[1]);
        assertEquals("single_observation", observations.translationDecisionReasons()[0]);
        assertEquals("single_observation", observations.translationDecisionReasons()[1]);
        assertEquals("single_megatag1_rotation", observations.rotationDecisionReasons()[0]);
        assertEquals("no_megatag1_rotation", observations.rotationDecisionReasons()[1]);
    }

    @Test
    void coalesceCorrelatedObservations_fallsBackToMegatag2RotationWithoutMegatag1() {
        VisionIO.PoseObservation megatag2Observation =
            new VisionIO.PoseObservation(
                1.000,
                new Pose3d(1.2, 2.3, 0.0, new Rotation3d(0.0, 0.0, 0.4)),
                0.0,
                2,
                2.5,
                VisionIO.PoseObservationType.MEGATAG_2
            );

        VisionUtil.CoalescedObservationsResult observations =
            VisionUtil.coalesceCorrelatedObservations(List.of(megatag2Observation));

        assertEquals(1, observations.observations().size());
        assertEquals(VisionIO.PoseObservationType.MEGATAG_2, observations.observations().get(0).type());
        assertEquals(VisionIO.PoseObservationType.MEGATAG_2, observations.observations().get(0).rotationType());
        assertEquals("MEGATAG_2", observations.rotationSourceTypes()[0]);
        assertEquals("no_megatag1_rotation", observations.rotationDecisionReasons()[0]);
    }

    @Test
    void getSelectionReason_usesHigherTagCountBeforeTypePreference() {
        VisionIO.PoseObservation megatag1Observation =
            new VisionIO.PoseObservation(
                1.000,
                new Pose3d(1.0, 2.0, 0.0, new Rotation3d()),
                0.4,
                3,
                4.0,
                VisionIO.PoseObservationType.MEGATAG_1
            );
        VisionIO.PoseObservation megatag2Observation =
            new VisionIO.PoseObservation(
                1.001,
                new Pose3d(1.1, 2.1, 0.0, new Rotation3d()),
                0.0,
                2,
                3.0,
                VisionIO.PoseObservationType.MEGATAG_2
            );

        assertEquals(
            "higher_tag_count",
            VisionUtil.getSelectionReason(megatag2Observation, megatag1Observation)
        );
    }
}
