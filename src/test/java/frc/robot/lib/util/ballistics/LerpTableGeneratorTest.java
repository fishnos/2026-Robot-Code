package frc.robot.lib.util.ballistics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.configs.ShooterConfig;
import java.util.List;
import org.junit.jupiter.api.Test;

class LerpTableGeneratorTest {
    @Test
    // Generated table should include all default distances and stay inside configured hood limits.
    void generate_producesDefaultDistanceEntriesWithReasonableValues() {
        ShooterConfig config = new ShooterConfig();
        config.shooterPoseZ = 0.95;
        config.hoodMinAngleDegrees = 15.0;
        config.hoodMaxAngleDegrees = 80.0;
        config.flywheelRadiusMeters = 0.05;
        config.backRollerGearRatio = 0.65;
        config.backRollerRadiusMeters = 0.03;

        List<LerpTableGenerator.LerpEntry> entries = LerpTableGenerator.generate(config);

        assertEquals(13, entries.size());

        double previousDistance = -1.0;
        for (LerpTableGenerator.LerpEntry entry : entries) {
            assertTrue(entry.distanceMeters > previousDistance);
            assertTrue(entry.hoodAngleDegrees >= config.hoodMinAngleDegrees);
            assertTrue(entry.hoodAngleDegrees <= config.hoodMaxAngleDegrees);
            assertTrue(entry.flywheelVelocityRPS > 0.0);
            previousDistance = entry.distanceMeters;
        }
    }
}
