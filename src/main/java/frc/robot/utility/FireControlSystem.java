package frc.robot.utility;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

import static edu.wpi.first.math.util.Units.*;

public class FireControlSystem {
    private static final InterpolatingTreeMap<Double, LookupEntry> DISTANCE_LOOKUP_TABLE = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            LookupEntry::interpolate
    );

    private static final double SHOOTER_VELOCITY_OFFSET = rotationsPerMinuteToRadiansPerSecond(0.0);

    static {
        DISTANCE_LOOKUP_TABLE.put(feetToMeters(1.0), new LookupEntry(degreesToRadians(50.0), rotationsPerMinuteToRadiansPerSecond(2500.0)));
        DISTANCE_LOOKUP_TABLE.put(2.0, new LookupEntry(degreesToRadians(40.0), rotationsPerMinuteToRadiansPerSecond(3000.0)));
        DISTANCE_LOOKUP_TABLE.put(2.5, new LookupEntry(degreesToRadians(32.0), rotationsPerMinuteToRadiansPerSecond(3250.0)));
        DISTANCE_LOOKUP_TABLE.put(3.0, new LookupEntry(degreesToRadians(27.0), rotationsPerMinuteToRadiansPerSecond(3250.0)));
        DISTANCE_LOOKUP_TABLE.put(3.5, new LookupEntry(degreesToRadians(21.0), rotationsPerMinuteToRadiansPerSecond(3500.0)));
        DISTANCE_LOOKUP_TABLE.put(4.0, new LookupEntry(degreesToRadians(18.0), rotationsPerMinuteToRadiansPerSecond(3750.0)));
        DISTANCE_LOOKUP_TABLE.put(4.5, new LookupEntry(degreesToRadians(14.0), rotationsPerMinuteToRadiansPerSecond(4250.0)));
    }

    public FiringSolution calculate(Pose2d currentPose, Translation2d goalPosition) {
        Translation2d delta = goalPosition.minus(currentPose.getTranslation());

        double distance = delta.getNorm();

        LookupEntry entry = DISTANCE_LOOKUP_TABLE.get(distance);

        Rotation2d rotation = delta.getAngle();

        return new FiringSolution(rotation, entry.pivotAngle, entry.shooterVelocity + SHOOTER_VELOCITY_OFFSET);
    }

    private record LookupEntry(double pivotAngle, double shooterVelocity) {
        public static LookupEntry interpolate(LookupEntry start, LookupEntry end, double t) {
            return new LookupEntry(
                    MathUtil.interpolate(start.pivotAngle, end.pivotAngle, t),
                    MathUtil.interpolate(start.shooterVelocity, end.shooterVelocity, t)
            );
        }
    }
}
