package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class AlignmentConstantsTest {
    @Test
    void isWithinBounds_returnsTrueForTowerLeftInteriorPoint() {
        Pose2d interiorPose = new Pose2d(2.0, 4.0, new Rotation2d());
        assertTrue(AlignmentConstants.Tower.isWithinBounds(interiorPose, AlignmentConstants.Tower.Left.BOUNDS));
    }

    @Test
    void isWithinBounds_returnsFalseForTowerLeftExteriorPoint() {
        Pose2d exteriorPose = new Pose2d(0.5, 4.0, new Rotation2d());
        assertFalse(AlignmentConstants.Tower.isWithinBounds(exteriorPose, AlignmentConstants.Tower.Left.BOUNDS));
    }
}
