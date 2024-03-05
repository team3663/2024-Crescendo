package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionMeasurement;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Drivetrain extends SubsystemBase {
    private static final double MAX_MEASUREMENT_DISTANCE = 1.0;

    private final DrivetrainIO io;
    private final DrivetrainInputsAutoLogged inputs = new DrivetrainInputsAutoLogged();
    private final Constants constants;

    public Drivetrain(DrivetrainIO io) {
        this.io = io;
        this.constants = io.getConstants();
        AutoBuilder.configureHolonomic(
                () -> inputs.pose,
                io::resetPose,
                () -> inputs.chassisSpeeds,
                io::drive,
                constants.pathFollowerConfig(),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public Constants getConstants() {
        return constants;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drivetrain", inputs);
    }

    public Rotation3d getRotation() {
        return inputs.rotation;
    }

    public Pose2d getPose() {
        return inputs.pose;
    }

    public void addVisionMeasurements(List<VisionMeasurement> measurements) {
        Translation2d currentPosition = inputs.pose.getTranslation();

        for (VisionMeasurement measurement : measurements) {
            Translation2d measuredPosition = measurement.estimatedPose().getTranslation();
            double distance = currentPosition.getDistance(measuredPosition);

            if (distance < MAX_MEASUREMENT_DISTANCE) {
                io.addVisionMeasurement(measurement.timestamp(), measurement.estimatedPose(), measurement.standardDeviation());
            }
        }
    }

    public Command drive(
            DoubleSupplier xVelocity,
            DoubleSupplier yVelocity,
            DoubleSupplier rotationalVelocity
    ) {
        return runEnd(
                // execute()
                () -> io.driveFieldOriented(
                        xVelocity.getAsDouble(),
                        yVelocity.getAsDouble(),
                        rotationalVelocity.getAsDouble()
                ),
                // end()
                io::stop
        );
    }

    public Command driveWithAngle(
            DoubleSupplier xVelocity,
            DoubleSupplier yVelocity,
            Supplier<Rotation2d> angle
    ) {
        return runEnd(
                // execute()
                () -> io.driveFieldOrientedFacingAngle(
                        xVelocity.getAsDouble(),
                        yVelocity.getAsDouble(),
                        angle.get()
                ),
                // end()
                io::stop
        );
    }

    public Command zeroGyroscope() {
        return Commands.runOnce(io::zeroGyroscope);
    }

    public record Constants(
            double maxModuleVelocity,
            double driveBaseRadius,
            HolonomicPathFollowerConfig pathFollowerConfig
    ) {
        public double maxTranslationalVelocity() {
            return maxModuleVelocity;
        }

        public double maxRotationalVelocity() {
            return maxModuleVelocity / driveBaseRadius;
        }
    }
}
