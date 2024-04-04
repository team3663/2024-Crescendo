package frc.robot.utility;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

public record FiringSolution(
        Optional<Rotation2d> robotRotation,
        double pivotAngle,
        Optional<Double> ampAngle,
        double shooterVelocity
) {
    public FiringSolution(double pivotAngle, double shooterVelocity) {
        this(Optional.empty(), pivotAngle, Optional.empty(), shooterVelocity);
    }

    public FiringSolution(Rotation2d robotRotation, double pivotAngle, double shooterVelocity) {
        this(Optional.of(robotRotation), pivotAngle, Optional.empty(), shooterVelocity);
    }

    public FiringSolution(Rotation2d robotRotation, double pivotAngle, double ampAngle, double shooterVelocity) {
        this(Optional.of(robotRotation), pivotAngle, Optional.of(ampAngle), shooterVelocity);
    }
}
