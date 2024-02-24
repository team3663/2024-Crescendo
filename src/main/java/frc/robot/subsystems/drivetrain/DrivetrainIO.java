package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.List;

import static edu.wpi.first.math.util.Units.feetToMeters;
import static edu.wpi.first.math.util.Units.inchesToMeters;

public interface DrivetrainIO {
    default Drivetrain.Constants getConstants() {
        double maxModuleVelocity = feetToMeters(16.0);
        double driveBaseRadius = inchesToMeters(24.0);

        return new Drivetrain.Constants(
                maxModuleVelocity,
                maxModuleVelocity / driveBaseRadius,
                new HolonomicPathFollowerConfig(maxModuleVelocity, driveBaseRadius, new ReplanningConfig())
        );
    }

    default void updateInputs(Inputs inputs) {}

    /**
     * Drives using a WPILib chassis speeds.
     *
     * @param chassisSpeeds The chassis speeds to follow.
     */
    default void drive(ChassisSpeeds chassisSpeeds) {}

    /**
     * Drives field-oriented with the ability to specify an X, Y, and rotational velocity.
     *
     * @param xVelocity          The target X (downfield) velocity in meters per second.
     * @param yVelocity          The target Y (toward the left side of the field) velocity in meters per second.
     * @param rotationalVelocity The target rotational (counter-clockwise positive) velocity in radians per second.
     */
    default void driveFieldOriented(double xVelocity, double yVelocity, double rotationalVelocity) {}

    /**
     * Drives field-oriented at a specified angle with the ability to specify an X, Y velocity.
     *
     * @param xVelocity The target X (downfield) velocity in meters per second.
     * @param yVelocity The target Y (toward the left side of the field) velocity in meters per second.
     * @param angle     The target angle to face while driving.
     */
    default void driveFieldOrientedFacingAngle(double xVelocity, double yVelocity, Rotation2d angle) {}

    /**
     * Stops the drivetrain.
     */
    default void stop() {}

    default void resetPose(Pose2d pose) {}
    default void zeroGyroscope() {}
    default void addVisionMeasurements(List<Pose3d> poses, List<Double> timestamps){}

    class Inputs implements LoggableInputs {
        public int successfulDaqs = 0;
        public int failedDaqs = 0;
        public double odometryPeriod = 0;

        public Pose2d pose = new Pose2d();

        public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

        public SwerveModuleState[] moduleStates = new SwerveModuleState[0];
        public SwerveModuleState[] moduleTargets = new SwerveModuleState[0];

        @Override
        public void toLog(LogTable table) {
            table.put("SuccessfulDaqs", successfulDaqs);
            table.put("FailedDaqs", failedDaqs);
            table.put("OdometryPeriod", odometryPeriod);
            if (pose != null)
                table.put("Pose", pose);
            if (moduleStates != null)
                table.put("ModuleStates", moduleStates);
            if (moduleTargets != null)
                table.put("ModuleTargets", moduleTargets);
        }

        @Override
        public void fromLog(LogTable table) {
            successfulDaqs = table.get("SuccessfulDaqs", 0);
            failedDaqs = table.get("FailedDaqs", 0);
            odometryPeriod = table.get("OdometryPeriod", 0.0);
            pose = table.get("Pose", new Pose2d());
            moduleStates = table.get("ModuleStates", new SwerveModuleState[0]);
            moduleTargets = table.get("ModuleTargets", new SwerveModuleState[0]);
        }
    }
}
