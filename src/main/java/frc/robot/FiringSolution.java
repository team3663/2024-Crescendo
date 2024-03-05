package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

public record FiringSolution(
        Optional<Rotation2d> robotRotation,
        double pivotAngle,
        double shooterVelocity
) {
    public FiringSolution(Rotation2d robotRotation, double pivotAngle, double shooterVelocity) {
        this(Optional.of(robotRotation), pivotAngle, shooterVelocity);
    }
}
