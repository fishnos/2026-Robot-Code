package frc.robot.testutil;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class TestLerpTableFactory {
    private TestLerpTableFactory() {}

    public static InterpolatingMatrixTreeMap<Double, N3, N1> table(double[]... rows) {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = new InterpolatingMatrixTreeMap<>();
        for (double[] row : rows) {
            if (row.length != 4) {
                throw new IllegalArgumentException("Each row must be [distance, hoodDeg, flywheelRps, tofSec]");
            }
            table.put(row[0], new Matrix<>(Nat.N3(), Nat.N1(), new double[] { row[1], row[2], row[3] }));
        }
        return table;
    }

    public static InterpolatingMatrixTreeMap<Double, N3, N1> constantRange(
        double minDistanceMeters,
        double maxDistanceMeters,
        double hoodAngleDegrees,
        double flywheelRps,
        double flightTimeSeconds
    ) {
        return table(
            new double[] { minDistanceMeters, hoodAngleDegrees, flywheelRps, flightTimeSeconds },
            new double[] { maxDistanceMeters, hoodAngleDegrees, flywheelRps, flightTimeSeconds }
        );
    }
}
