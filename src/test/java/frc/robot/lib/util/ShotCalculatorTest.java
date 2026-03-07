package frc.robot.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.lib.util.ShotCalculator.ShotData;
import frc.robot.testutil.TestLerpTableFactory;
import org.junit.jupiter.api.Test;

class ShotCalculatorTest {
    private static final double EPS = 1e-9;

    @Test
    // At the table setpoint, visualization exit velocity should match the direct wheel model.
    void calculateExitVelocityMetersPerSec_atSetpointMatchesWheelModel() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            10.0,
            45.0,
            30.0,
            0.85
        );

        double actual = ShotCalculator.calculateExitVelocityMetersPerSec(
            4.0,
            table,
            30.0,
            rps -> rps * 0.2
        );

        assertEquals(6.0, actual, EPS);
    }

    @Test
    // Measured flywheel RPS should scale visualization exit velocity by measured/setpoint.
    void calculateExitVelocityMetersPerSec_scalesByMeasuredVsSetpointRps() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.table(
            new double[] { 3.0, 40.0, 30.0, 0.75 },
            new double[] { 5.0, 50.0, 40.0, 0.90 }
        );

        double distance = 4.0;
        double setpointRps = table.get(distance).get(1, 0);
        double full = ShotCalculator.calculateExitVelocityMetersPerSec(
            distance,
            table,
            setpointRps,
            rps -> rps * 0.2
        );
        double half = ShotCalculator.calculateExitVelocityMetersPerSec(
            distance,
            table,
            setpointRps * 0.5,
            rps -> rps * 0.2
        );

        assertEquals(0.5, half / full, 1e-6);
    }

    @Test
    // Visualization backspin should scale by the same measured/setpoint ratio.
    void calculateSpinRateRadPerSec_scalesByMeasuredVsSetpointRps() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.table(
            new double[] { 3.0, 42.0, 30.0, 0.78 },
            new double[] { 5.0, 52.0, 40.0, 0.93 }
        );

        double distance = 4.0;
        double measuredRps = 32.0;
        double spin = ShotCalculator.calculateSpinRateRadPerSec(
            distance,
            table,
            measuredRps,
            rps -> rps * 5.0
        );

        assertEquals(measuredRps * 5.0, spin, EPS);
    }

    @Test
    // Hood and TOF stay on the aiming path; visualization exit velocity depends only on flywheel RPS.
    void calculateExitVelocityMetersPerSec_ignoresHoodAndFlightTimeColumnsWhenSetpointRpsMatches() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.table(
            new double[] { 2.0, 35.0, 25.0, 0.50 },
            new double[] { 4.0, 60.0, 25.0, 1.10 }
        );

        double atTwoMeters = ShotCalculator.calculateExitVelocityMetersPerSec(
            2.0,
            table,
            20.0,
            rps -> rps * 0.5
        );
        double atFourMeters = ShotCalculator.calculateExitVelocityMetersPerSec(
            4.0,
            table,
            20.0,
            rps -> rps * 0.5
        );

        assertEquals(atTwoMeters, atFourMeters, EPS);
    }

    @Test
    // Hood and TOF stay on the aiming path; visualization backspin depends only on flywheel RPS.
    void calculateSpinRateRadPerSec_ignoresHoodAndFlightTimeColumnsWhenSetpointRpsMatches() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.table(
            new double[] { 2.0, 35.0, 25.0, 0.50 },
            new double[] { 4.0, 60.0, 25.0, 1.10 }
        );

        double atTwoMeters = ShotCalculator.calculateSpinRateRadPerSec(
            2.0,
            table,
            20.0,
            rps -> rps * 4.0
        );
        double atFourMeters = ShotCalculator.calculateSpinRateRadPerSec(
            4.0,
            table,
            20.0,
            rps -> rps * 4.0
        );

        assertEquals(atTwoMeters, atFourMeters, EPS);
    }

    @Test
    // Zero setpoint RPS should safely return zero exit velocity.
    void calculateExitVelocityMetersPerSec_whenSetpointRpsIsZero_returnsZero() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            10.0,
            30.0,
            0.0,
            1.0
        );

        double actual = ShotCalculator.calculateExitVelocityMetersPerSec(
            1.0,
            table,
            20.0,
            rps -> rps * 0.5
        );
        assertEquals(0.0, actual, EPS);
    }

    @Test
    // Non-finite wheel-model outputs should be clamped away instead of propagating NaN/Inf.
    void calculateShotKinematics_whenWheelModelReturnsNonFinite_clampsToZero() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            10.0,
            45.0,
            20.0,
            0.75
        );

        double exitVelocity = ShotCalculator.calculateExitVelocityMetersPerSec(
            2.0,
            table,
            20.0,
            rps -> Double.NaN
        );
        double spinRate = ShotCalculator.calculateSpinRateRadPerSec(
            2.0,
            table,
            20.0,
            rps -> Double.POSITIVE_INFINITY
        );

        assertEquals(0.0, exitVelocity, EPS);
        assertEquals(0.0, spinRate, EPS);
    }

    @Test
    // With no robot motion, field yaw and compensated target should point directly at the goal.
    void calculate_zeroRobotVelocity_aimsDirectlyAtTarget() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            20.0,
            45.0,
            30.0,
            1.0
        );

        ShotData shot = ShotCalculator.calculate(
            new Translation3d(5.0, 1.0, 2.0),
            new Translation3d(1.0, 1.0, 1.0),
            new ChassisSpeeds(),
            table,
            0.0,
            30.0,
            rps -> rps,
            rps -> 0.0,
            new Translation2d(),
            new Rotation2d()
        );

        assertEquals(0.0, shot.targetFieldYaw().getRadians(), EPS);
        assertEquals(5.0, shot.compensatedTargetPosition().getX(), EPS);
        assertEquals(1.0, shot.compensatedTargetPosition().getY(), EPS);
        assertEquals(4.0, shot.effectiveDistance(), EPS);
    }

    @Test
    // Forward robot translation should shift compensated target backward by travel during flight.
    void calculate_withRobotTranslation_compensatesTargetPosition() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            20.0,
            45.0,
            30.0,
            1.0
        );

        ShotData shot = ShotCalculator.calculate(
            new Translation3d(10.0, 0.0, 2.0),
            new Translation3d(0.0, 0.0, 1.0),
            new ChassisSpeeds(2.0, 0.0, 0.0),
            table,
            0.0,
            30.0,
            rps -> rps,
            rps -> 0.0,
            new Translation2d(),
            new Rotation2d()
        );

        assertEquals(8.0, shot.compensatedTargetPosition().getX(), EPS);
        assertEquals(0.0, shot.compensatedTargetPosition().getY(), EPS);
        assertEquals(8.0, shot.effectiveDistance(), EPS);
    }

    @Test
    // Latency term should add additional displacement compensation beyond flight-time compensation.
    void calculate_withLatency_compensatesAdditionalDisplacement() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            20.0,
            45.0,
            30.0,
            1.0
        );

        ShotData noLatency = ShotCalculator.calculate(
            new Translation3d(10.0, 0.0, 2.0),
            new Translation3d(0.0, 0.0, 1.0),
            new ChassisSpeeds(1.0, 0.0, 0.0),
            table,
            0.0,
            30.0,
            rps -> rps,
            rps -> 0.0,
            new Translation2d(),
            new Rotation2d()
        );
        ShotData withLatency = ShotCalculator.calculate(
            new Translation3d(10.0, 0.0, 2.0),
            new Translation3d(0.0, 0.0, 1.0),
            new ChassisSpeeds(1.0, 0.0, 0.0),
            table,
            0.5,
            30.0,
            rps -> rps,
            rps -> 0.0,
            new Translation2d(),
            new Rotation2d()
        );

        assertEquals(9.0, noLatency.compensatedTargetPosition().getX(), EPS);
        assertEquals(8.5, withLatency.compensatedTargetPosition().getX(), EPS);
        assertTrue(withLatency.compensatedTargetPosition().getX() < noLatency.compensatedTargetPosition().getX());
    }

    @Test
    // Robot omega with shooter offset should contribute tangential compensation.
    void calculate_withRobotOmegaAndShooterOffset_compensatesUsingTangentialVelocity() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            20.0,
            45.0,
            30.0,
            1.0
        );

        ShotData shot = ShotCalculator.calculate(
            new Translation3d(0.0, 10.0, 2.0),
            new Translation3d(0.0, 0.0, 1.0),
            new ChassisSpeeds(0.0, 0.0, 1.0),
            table,
            0.0,
            30.0,
            rps -> rps,
            rps -> 0.0,
            new Translation2d(1.0, 0.0),
            new Rotation2d()
        );

        assertEquals(9.0, shot.compensatedTargetPosition().getY(), EPS);
        assertTrue(shot.compensatedTargetPosition().getY() < 10.0);
    }

    @Test
    // Lower measured flywheel speed should proportionally reduce final visualization exit velocity.
    void calculate_lowerMeasuredFlywheelRps_reducesExitVelocityProportionally() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            20.0,
            50.0,
            30.0,
            1.0
        );

        ShotData fullSpeed = ShotCalculator.calculate(
            new Translation3d(6.0, 0.0, 2.0),
            new Translation3d(0.0, 0.0, 1.0),
            new ChassisSpeeds(),
            table,
            0.0,
            30.0,
            rps -> rps * 0.2,
            rps -> rps * 5.0,
            new Translation2d(),
            new Rotation2d()
        );
        ShotData halfSpeed = ShotCalculator.calculate(
            new Translation3d(6.0, 0.0, 2.0),
            new Translation3d(0.0, 0.0, 1.0),
            new ChassisSpeeds(),
            table,
            0.0,
            15.0,
            rps -> rps * 0.2,
            rps -> rps * 5.0,
            new Translation2d(),
            new Rotation2d()
        );

        assertEquals(0.5, halfSpeed.exitVelocity() / fullSpeed.exitVelocity(), EPS);
    }
}
