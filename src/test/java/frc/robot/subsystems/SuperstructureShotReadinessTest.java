package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class SuperstructureShotReadinessTest {
    @Test
    // Ready when both impact error and distance bounds are satisfied.
    void isShotReady_trueWhenImpactAndDistanceAreValid() {
        boolean ready = Superstructure.isShotReady(0.05, 0.3, 4.0, 2.0, 6.0);
        assertTrue(ready);
    }

    @Test
    // Exceeding impact tolerance should fail readiness.
    void isShotReady_falseWhenImpactExceedsTolerance() {
        boolean ready = Superstructure.isShotReady(0.31, 0.3, 4.0, 2.0, 6.0);
        assertFalse(ready);
    }

    @Test
    // Distances below configured minimum should fail readiness.
    void isShotReady_falseWhenDistanceBelowMinimum() {
        boolean ready = Superstructure.isShotReady(0.05, 0.3, 1.99, 2.0, 6.0);
        assertFalse(ready);
    }

    @Test
    // Distances above configured maximum should fail readiness.
    void isShotReady_falseWhenDistanceAboveMaximum() {
        boolean ready = Superstructure.isShotReady(0.05, 0.3, 6.01, 2.0, 6.0);
        assertFalse(ready);
    }

    @Test
    // Min/max boundaries are inclusive.
    void isShotReady_allowsBoundaryDistances() {
        assertTrue(Superstructure.isShotReady(0.05, 0.3, 2.0, 2.0, 6.0));
        assertTrue(Superstructure.isShotReady(0.05, 0.3, 6.0, 2.0, 6.0));
    }
}
