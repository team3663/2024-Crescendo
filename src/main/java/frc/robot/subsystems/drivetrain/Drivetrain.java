package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionMeasurement;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Drivetrain extends SubsystemBase {
    private final DrivetrainIO io;
    private final DrivetrainInputsAutoLogged inputs = new DrivetrainInputsAutoLogged();
    private final Constants constants;

    private static Pose2d mirrorIfRed(Pose2d pose) {
        boolean shouldMirror =
                DriverStation.getAlliance()
                        .map(alliance -> alliance == DriverStation.Alliance.Red)
                        .orElse(false);

        if (shouldMirror) {
            return new Pose2d(
                    new Translation2d(frc.robot.Constants.FIELD_LAYOUT.getFieldLength() - pose.getX(),
                            pose.getY()),
                    Rotation2d.fromDegrees(180.0).rotateBy(pose.getRotation())
            );
        } else {
            return pose;
        }
    }

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
//        AutoBuilder.configureHolonomic(
//                () -> mirrorIfRed(inputs.pose),
//                pose -> io.resetPose(mirrorIfRed(pose)),
//                () -> inputs.chassisSpeeds,
//                io::drive,
//                constants.pathFollowerConfig(),
//                () -> false,
//                this // Reference to this subsystem to set requirements
//        );

        PathPlannerLogging.setLogCurrentPoseCallback(pose -> Logger.recordOutput("Drivetrain/PPCurrent", pose));
        PathPlannerLogging.setLogTargetPoseCallback(pose -> Logger.recordOutput("Drivetrain/PPTarget", pose));
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
            io.addVisionMeasurement(measurement.timestamp(), measurement.estimatedPose(), measurement.standardDeviation());
        }
    }

    public Command drive(
            DoubleSupplier xVelocitySupplier,
            DoubleSupplier yVelocitySupplier,
            DoubleSupplier angularVelocitySupplier
    ) {
        return runEnd(
                // execute()
                () -> io.driveFieldOriented(
                        xVelocitySupplier.getAsDouble(),
                        yVelocitySupplier.getAsDouble(),
                        angularVelocitySupplier.getAsDouble()
                ),
                // end()
                io::stop
        );
    }

    public Command driveWithAngle(
            DoubleSupplier xVelocitySupplier,
            DoubleSupplier yVelocitySupplier,
            Supplier<Rotation2d> angleSupplier
    ) {
        return runEnd(
                // execute()
                () -> io.driveFieldOrientedFacingAngle(
                        xVelocitySupplier.getAsDouble(),
                        yVelocitySupplier.getAsDouble(),
                        angleSupplier.get()
                ),
                // end()
                io::stop
        );
    }

    public Command driveWithOptionalAngle(
            DoubleSupplier xVelocitySupplier,
            DoubleSupplier yVelocitySupplier,
            DoubleSupplier angularVelocitySupplier,
            Supplier<Optional<Rotation2d>> angleSupplier
    ) {
        ProfiledPIDController controller = new ProfiledPIDController(10.0, 0.0, 0.0,
                new TrapezoidProfile.Constraints(0.9 * constants.maxAngularVelocity(), Units.degreesToRadians(720.0)));
        controller.enableContinuousInput(-Math.PI, Math.PI);

        return runEnd(
                // execute()
                () -> {
                    double xVelocity = xVelocitySupplier.getAsDouble();
                    double yVelocity = yVelocitySupplier.getAsDouble();
                    double angularVelocity = angularVelocitySupplier.getAsDouble();
                    Optional<Rotation2d> angle = angleSupplier.get();

                    // If we were given an angle, drive while facing the angle
                    if (angle.isPresent()) {
                        // For some reason, driving with a fixed angle is not working
                        // Instead of that, use a PID controller and use the output as the angular velocity
                        // for field oriented driving
//                        io.driveFieldOrientedFacingAngle(xVelocity, yVelocity, angle.get());
                        angularVelocity = controller.calculate(inputs.pose.getRotation().getRadians(), angle.get().getRadians());
                    }

                    io.driveFieldOriented(xVelocity, yVelocity, angularVelocity);
                },
                // end()
                io::stop
        ).beforeStarting(() -> controller.reset(inputs.pose.getRotation().getRadians()));
    }

    public Command zeroGyroscope() {
        return Commands.runOnce(io::zeroGyroscope);
    }

    public Command driveRobotOriented(
            DoubleSupplier xVelocitySupplier,
            DoubleSupplier yVelocitySupplier,
            DoubleSupplier angularVelocitySupplier) {
        return runEnd(
                // execute()
                () -> io.driveRobotOriented(
                        xVelocitySupplier.getAsDouble(),
                        yVelocitySupplier.getAsDouble(),
                        angularVelocitySupplier.getAsDouble()
                ),
                // end()
                io::stop
        );
    }

    public record Constants(
            double maxModuleVelocity,
            double driveBaseRadius,
            HolonomicPathFollowerConfig pathFollowerConfig
    ) {
        public double maxLinearVelocity() {
            return maxModuleVelocity;
        }

        public double maxAngularVelocity() {
            return maxModuleVelocity / driveBaseRadius;
        }
    }
}
