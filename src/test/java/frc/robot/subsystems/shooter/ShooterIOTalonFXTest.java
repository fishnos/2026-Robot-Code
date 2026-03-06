package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.constants.Constants;
import org.junit.jupiter.api.Test;

class ShooterIOTalonFXTest {
    private static final double EPS = 1e-9;

    @Test
    void calculateTurretMotionMagicCommand_usesRequestedVelocityWhenProvided() {
        ShooterIOTalonFX.TurretMotionMagicCommand command = ShooterIOTalonFX.calculateTurretMotionMagicCommand(
            0.4,
            1.2,
            0.35,
            1.0,
            0.0,
            1.0,
            5.0,
            50.0,
            Constants.kLOOP_CYCLE_MS
        );

        assertEquals(0.4, command.positionRotations(), EPS);
        assertEquals(1.2, command.desiredVelocityRotPerSec(), EPS);
        assertEquals(1.2, command.commandedVelocityRotPerSec(), EPS);
        assertEquals(10.0, command.commandedAccelerationRotPerSec2(), EPS);
        assertEquals(1.2, command.motionMagicVelocityRotPerSec(), EPS);
        assertEquals(10.0, command.motionMagicAccelerationRotPerSec2(), EPS);
    }

    @Test
    void calculateTurretMotionMagicCommand_clampsVelocityStepByMaxAcceleration() {
        ShooterIOTalonFX.TurretMotionMagicCommand command = ShooterIOTalonFX.calculateTurretMotionMagicCommand(
            0.4,
            10.0,
            0.35,
            1.0,
            0.0,
            1.0,
            4.0,
            20.0,
            Constants.kLOOP_CYCLE_MS
        );

        assertEquals(10.0, command.desiredVelocityRotPerSec(), EPS);
        assertEquals(1.4, command.commandedVelocityRotPerSec(), EPS);
        assertEquals(20.0, command.commandedAccelerationRotPerSec2(), EPS);
    }

    @Test
    void calculateTurretMotionMagicCommand_derivesVelocityFromCurrentPositionErrorWhenUnspecified() {
        ShooterIOTalonFX.TurretMotionMagicCommand command = ShooterIOTalonFX.calculateTurretMotionMagicCommand(
            0.40,
            Double.NaN,
            0.35,
            0.0,
            0.0,
            1.0,
            5.0,
            100.0,
            Constants.kLOOP_CYCLE_MS
        );

        assertEquals(2.5, command.desiredVelocityRotPerSec(), EPS);
        assertEquals(2.0, command.commandedVelocityRotPerSec(), EPS);
        assertEquals(100.0, command.commandedAccelerationRotPerSec2(), EPS);
    }

    @Test
    void calculateTurretMotionMagicCommand_usesAbsoluteMagnitudesForMotionMagicRequest() {
        ShooterIOTalonFX.TurretMotionMagicCommand command = ShooterIOTalonFX.calculateTurretMotionMagicCommand(
            0.2,
            -1.0,
            0.25,
            -0.6,
            0.0,
            1.0,
            5.0,
            50.0,
            Constants.kLOOP_CYCLE_MS
        );

        assertEquals(-1.0, command.commandedVelocityRotPerSec(), EPS);
        assertEquals(-20.0, command.commandedAccelerationRotPerSec2(), EPS);
        assertEquals(1.0, command.motionMagicVelocityRotPerSec(), EPS);
        assertEquals(20.0, command.motionMagicAccelerationRotPerSec2(), EPS);
    }

    @Test
    void calculateTurretMotionMagicCommand_heldSetpointDoesNotCollapseCruiseVelocityToZeroWhileErrorRemains() {
        ShooterIOTalonFX.TurretMotionMagicCommand command = ShooterIOTalonFX.calculateTurretMotionMagicCommand(
            0.40,
            Double.NaN,
            0.30,
            0.0,
            0.0,
            1.0,
            5.0,
            100.0,
            Constants.kLOOP_CYCLE_MS
        );

        assertEquals(5.0, command.desiredVelocityRotPerSec(), EPS);
        assertEquals(2.0, command.commandedVelocityRotPerSec(), EPS);
        assertEquals(100.0, command.commandedAccelerationRotPerSec2(), EPS);
    }
}
