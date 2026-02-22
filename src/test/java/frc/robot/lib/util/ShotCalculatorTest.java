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
import java.util.function.DoubleUnaryOperator;
import org.junit.jupiter.api.Test;

class ShotCalculatorTest {
    private static final double EPS = 1e-9;

    @Test
    // Exit velocity should equal TOF base velocity scaled by measured/setpoint wheel-model ratio.
    void calculateExitVelocityMetersPerSec_scalesByMeasuredVsSetpointExitVelocity() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.table(
            new double[] { 3.0, 40.0, 30.0, 0.45 },
            new double[] { 5.0, 50.0, 40.0, 0.60 }
        );
        DoubleUnaryOperator wheelModel = rps -> rps * 0.2;

        double distance = 4.0;
        double measuredRps = 28.0;
        double actual = ShotCalculator.calculateExitVelocityMetersPerSec(distance, table, measuredRps, wheelModel);

        double hoodDeg = table.get(distance).get(0, 0);
        double setpointRps = table.get(distance).get(1, 0);
        double tof = table.get(distance).get(2, 0);
        double base = distance / (tof * Math.abs(Math.cos(Math.toRadians(hoodDeg))));
        double expected = base * (wheelModel.applyAsDouble(measuredRps) / wheelModel.applyAsDouble(setpointRps));

        assertEquals(expected, actual, EPS);
    }

    @Test
    // Near-vertical hood angles should use the cosine floor to avoid singular speeds.
    void calculateExitVelocityMetersPerSec_clampsCosineNearVertical() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            10.0,
            90.0,
            30.0,
            0.5
        );

        double actual = ShotCalculator.calculateExitVelocityMetersPerSec(2.0, table, 30.0, rps -> rps);
        double expected = 2.0 / (0.5 * 0.05);

        assertEquals(expected, actual, EPS);
    }

    @Test
    // Zero/invalid TOF should use the minimum flight-time floor.
    void calculateExitVelocityMetersPerSec_clampsFlightTimeAtMinimum() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            10.0,
            45.0,
            20.0,
            0.0
        );

        double actual = ShotCalculator.calculateExitVelocityMetersPerSec(1.0, table, 20.0, rps -> rps);
        double expected = 1.0 / (1e-3 * Math.abs(Math.cos(Math.toRadians(45.0))));

        assertEquals(expected, actual, EPS);
    }

    @Test
    // If wheel model reports zero setpoint exit velocity, scaled exit velocity should collapse to zero.
    void calculateExitVelocityMetersPerSec_whenSetpointExitVelocityIsZero_returnsZero() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            10.0,
            45.0,
            20.0,
            0.4
        );

        double actual = ShotCalculator.calculateExitVelocityMetersPerSec(2.0, table, 20.0, rps -> 0.0);

        assertEquals(0.0, actual, EPS);
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
            new Translation2d(1.0, 0.0),
            new Rotation2d()
        );

        assertEquals(9.0, shot.compensatedTargetPosition().getY(), EPS);
        assertTrue(shot.compensatedTargetPosition().getY() < 10.0);
    }

    @Test
    // Lower measured flywheel speed should proportionally reduce final exit velocity.
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
            new Translation2d(),
            new Rotation2d()
        );

        assertEquals(0.5, halfSpeed.exitVelocity() / fullSpeed.exitVelocity(), EPS);
    }
}
