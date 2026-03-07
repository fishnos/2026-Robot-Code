package frc.robot.configs;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.google.gson.Gson;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class ShooterConfigTest {
    @Test
    void compConfig_includesTurretConfiguration() throws IOException {
        ShooterConfig config = new Gson()
            .fromJson(Files.readString(Path.of("src/main/deploy/configs/shooter/comp.json")), ShooterConfig.class);

        assertTrue(config.turretCanId > 0);
        assertTrue(config.turretStatorCurrentLimit > 0.0);
        assertTrue(config.turretMotorToOutputShaftRatio > 0.0);
        assertTrue(config.turretMaxAngleDeg > config.turretMinAngleDeg);
        assertTrue(config.turretMaxVelocityDegPerSec > 0.0);
        assertTrue(config.turretMaxAccelerationDegPerSec2 > 0.0);
        assertTrue(config.turretMaxJerkDegPerSec3 > 0.0);
        assertTrue(config.turretAngleToleranceRotations > 0.0);
    }
}
