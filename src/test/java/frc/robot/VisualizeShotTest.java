package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import frc.robot.lib.util.ballistics.ProjectileVisualizer;
import java.lang.reflect.Field;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class VisualizeShotTest {
    @BeforeEach
    void setUp() {
        ProjectileVisualizer.clearProjectiles();
    }

    @Test
    // Explicit launch velocity constructor should still push one projectile into the visualizer.
    void explicitVelocityConstructor_addsProjectile() {
        int before = getActiveProjectileCountReflective();

        assertDoesNotThrow(() -> new VisualizeShot(9.0));

        int after = getActiveProjectileCountReflective();
        assertEquals(before + 1, after);
    }

    @Test
    // Default constructor should also register one projectile.
    void defaultConstructor_addsProjectile() {
        int before = getActiveProjectileCountReflective();

        assertDoesNotThrow(() -> new VisualizeShot());

        int after = getActiveProjectileCountReflective();
        assertEquals(before + 1, after);
    }

    @SuppressWarnings("unchecked")
    private int getActiveProjectileCountReflective() {
        try {
            Field projectilesField = ProjectileVisualizer.class.getDeclaredField("projectiles");
            projectilesField.setAccessible(true);
            List<Object> projectiles = (List<Object>) projectilesField.get(ProjectileVisualizer.getInstance());
            return projectiles.size();
        } catch (ReflectiveOperationException e) {
            throw new AssertionError("Failed to inspect projectile count for test", e);
        }
    }
}
