package frc.robot.utility;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Collection;

public record SubwooferRotations(
        Rotation2d front,
        Rotation2d ampSide,
        Rotation2d sourceSide
) {
}
