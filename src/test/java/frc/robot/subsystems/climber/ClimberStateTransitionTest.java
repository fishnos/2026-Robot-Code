package frc.robot.subsystems.climber;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.configs.ClimberConfig;
import org.junit.jupiter.api.Test;

class ClimberStateTransitionTest {
    @Test
    void resolveActiveState_routesRetractedToClimbedThroughExtended() {
        assertEquals(
            Climber.DesiredState.EXTENDED,
            Climber.resolveActiveState(Climber.CurrentState.RETRACTED, Climber.DesiredState.CLIMBED)
        );
        assertEquals(
            Climber.DesiredState.CLIMBED,
            Climber.resolveActiveState(Climber.CurrentState.EXTENDED, Climber.DesiredState.CLIMBED)
        );
    }

    @Test
    void resolveActiveState_routesClimbedToRetractedThroughExtended() {
        assertEquals(
            Climber.DesiredState.EXTENDED,
            Climber.resolveActiveState(Climber.CurrentState.CLIMBED, Climber.DesiredState.RETRACTED)
        );
        assertEquals(
            Climber.DesiredState.RETRACTED,
            Climber.resolveActiveState(Climber.CurrentState.EXTENDED, Climber.DesiredState.RETRACTED)
        );
    }

    @Test
    void resolveActiveState_routesClimbedToExtendedThroughExtended() {
        assertEquals(
            Climber.DesiredState.EXTENDED,
            Climber.resolveActiveState(Climber.CurrentState.CLIMBED, Climber.DesiredState.EXTENDED)
        );
    }

    @Test
    void resolveActiveState_leavesNonClimbTransitionsDirect() {
        assertEquals(
            Climber.DesiredState.EXTENDED,
            Climber.resolveActiveState(Climber.CurrentState.RETRACTED, Climber.DesiredState.EXTENDED)
        );
        assertEquals(
            Climber.DesiredState.RETRACTED,
            Climber.resolveActiveState(Climber.CurrentState.EXTENDED, Climber.DesiredState.RETRACTED)
        );
    }

    @Test
    void resolveActiveState_keepsClimbedWhenAlreadyClimbed() {
        assertEquals(
            Climber.DesiredState.CLIMBED,
            Climber.resolveActiveState(Climber.CurrentState.CLIMBED, Climber.DesiredState.CLIMBED)
        );
    }

    @Test
    void climbedSetpoint_usesDedicatedProfileWhileOthersUseDefaultProfile() {
        ClimberConfig config = new ClimberConfig();
        config.climberHomePositionRotations = 0.0;
        config.climberReadyPositionRotations = 8.0;
        config.climberLiftedPositionRotations = 2.0;
        config.climberDefaultProfileMaxVelocityRotationsPerSec = 4.0;
        config.climberDefaultProfileMaxAccelerationRotationsPerSec2 = 8.0;
        config.climberClimbingProfileMaxVelocityRotationsPerSec = 1.5;
        config.climberClimbingProfileMaxAccelerationRotationsPerSec2 = 3.0;

        assertEquals(4.0, Climber.ClimberSetpoint.RETRACTED.maxVelocityRotationsPerSec(config));
        assertEquals(8.0, Climber.ClimberSetpoint.RETRACTED.maxAccelerationRotationsPerSec2(config));
        assertEquals(4.0, Climber.ClimberSetpoint.EXTENDED.maxVelocityRotationsPerSec(config));
        assertEquals(8.0, Climber.ClimberSetpoint.EXTENDED.maxAccelerationRotationsPerSec2(config));
        assertEquals(1.5, Climber.ClimberSetpoint.CLIMBED.maxVelocityRotationsPerSec(config));
        assertEquals(3.0, Climber.ClimberSetpoint.CLIMBED.maxAccelerationRotationsPerSec2(config));
    }
}
