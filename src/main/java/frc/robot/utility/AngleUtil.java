package frc.robot.utility;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class AngleUtil {
    public static double angleDifference(Rotation2d lhs, Rotation2d rhs) {
        return Math.abs(MathUtil.angleModulus(lhs.minus(rhs).getRadians()));
    }
}
