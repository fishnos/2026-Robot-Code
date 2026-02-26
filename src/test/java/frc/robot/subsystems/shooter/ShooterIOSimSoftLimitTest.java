package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ShooterIOSimSoftLimitTest {
    private static final double EPS = 1e-9;

    @Test
    void applySoftLimit_clampsBelowMinAndStopsFurtherNegativeVelocity() {
        ShooterIOSim.SoftLimitedState state = ShooterIOSim.applySoftLimit(-1.5, -3.0, -1.25, 1.75);
        assertEquals(-1.25, state.positionRotations(), EPS);
        assertEquals(0.0, state.velocityRotationsPerSec(), EPS);
    }

    @Test
    void applySoftLimit_clampsAboveMaxAndStopsFurtherPositiveVelocity() {
        ShooterIOSim.SoftLimitedState state = ShooterIOSim.applySoftLimit(2.2, 4.0, -1.25, 1.75);
        assertEquals(1.75, state.positionRotations(), EPS);
        assertEquals(0.0, state.velocityRotationsPerSec(), EPS);
    }

    @Test
    void applySoftLimit_allowsVelocityBackIntoRangeAtMinLimit() {
        ShooterIOSim.SoftLimitedState state = ShooterIOSim.applySoftLimit(-1.5, 2.0, -1.25, 1.75);
        assertEquals(-1.25, state.positionRotations(), EPS);
        assertEquals(2.0, state.velocityRotationsPerSec(), EPS);
    }

    @Test
    void applySoftLimit_leavesInRangeStateUnchanged() {
        ShooterIOSim.SoftLimitedState state = ShooterIOSim.applySoftLimit(0.4, -0.3, -1.25, 1.75);
        assertEquals(0.4, state.positionRotations(), EPS);
        assertEquals(-0.3, state.velocityRotationsPerSec(), EPS);
    }
}
