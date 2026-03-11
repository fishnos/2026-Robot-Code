package frc.robot.lib.util.ballistics;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ProjectileVisualizerTest {
    private final ProjectileVisualizer visualizer = ProjectileVisualizer.getInstance();
    private final AtomicReference<Double> nowSec = new AtomicReference<>(0.0);

    @BeforeEach
    void setUp() {
        visualizer.setTimestampSupplierForTesting(() -> nowSec.get());
        ProjectileVisualizer.clearProjectiles();
    }

    @AfterEach
    void tearDown() {
        ProjectileVisualizer.clearProjectiles();
        visualizer.resetTimestampSupplierForTesting();
    }

    @Test
    // IDs should increment while active and reset after clear.
    void addProjectile_returnsIncrementingIds_andClearResetsCounter() {
        int firstId = ProjectileVisualizer.addProjectile(
            0.0,
            0.0,
            0.0,
            new Pose3d(0.0, 0.0, 1.0, new Rotation3d()),
            0.0
        );
        int secondId = ProjectileVisualizer.addProjectile(
            0.0,
            0.0,
            0.0,
            new Pose3d(0.0, 0.0, 1.0, new Rotation3d()),
            0.0
        );

        assertEquals(0, firstId);
        assertEquals(1, secondId);

        ProjectileVisualizer.clearProjectiles();

        int resetId = ProjectileVisualizer.addProjectile(
            0.0,
            0.0,
            0.0,
            new Pose3d(0.0, 0.0, 1.0, new Rotation3d()),
            0.0
        );
        assertEquals(0, resetId);
    }

    @Test
    // Projectile already below final height threshold should be removed on periodic update.
    void periodic_removesFinishedProjectile() {
        ProjectileVisualizer.addProjectile(
            0.0,
            0.0,
            0.0,
            new Pose3d(0.0, 0.0, 1.0, new Rotation3d()),
            2.0
        );
        assertEquals(1, visualizer.getActiveProjectileCountForTesting());

        nowSec.set(0.0);
        visualizer.periodic();

        assertEquals(0, visualizer.getActiveProjectileCountForTesting());
    }

    @Test
    // Valid in-flight projectile should remain active across short periodic steps.
    void periodic_keepsActiveProjectileWhileInFlight() {
        ProjectileVisualizer.addProjectile(
            0.0,
            0.0,
            12.0,
            new Pose3d(0.0, 0.0, 1.0, new Rotation3d(0.0, Math.toRadians(45.0), 0.0)),
            0.0
        );

        nowSec.set(0.0);
        visualizer.periodic();
        assertEquals(1, visualizer.getActiveProjectileCountForTesting());

        nowSec.set(0.05);
        visualizer.periodic();
        assertEquals(1, visualizer.getActiveProjectileCountForTesting());
    }

    @Test
    // Null launch pose supplier should fall back safely without throwing.
    void addProjectile_withNullLaunchPoseSupplier_doesNotThrow() {
        assertDoesNotThrow(() -> {
            ProjectileVisualizer.addProjectile(
                () -> 0.0,
                () -> 0.0,
                () -> 8.0,
                () -> null,
                () -> 0.0,
                () -> 1000.0
            );
            nowSec.set(0.0);
            visualizer.periodic();
        });
    }
}
