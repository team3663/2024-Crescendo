package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;

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

    default void updateInputs(DrivetrainInputs inputs) {
    }

    /**
     * Drives using a WPILib chassis speeds.
     *
     * @param chassisSpeeds The chassis speeds to follow.
     */
    default void drive(ChassisSpeeds chassisSpeeds) {
    }

    /**
     * Drives field-oriented with the ability to specify an X, Y, and rotational velocity.
     *
     * @param xVelocity       The target X (downfield) velocity in meters per second.
     * @param yVelocity       The target Y (toward the left side of the field) velocity in meters per second.
     * @param angularVelocity The target angular (counter-clockwise positive) velocity in radians per second.
     */
    default void driveFieldOriented(double xVelocity, double yVelocity, double angularVelocity) {
    }

    /**
     * Drives robot-oriented with the ability to specify an X, Y, and rotational velocity.
     *
     * @param xVelocity       The target X (forward) velocity in meters per second.
     * @param yVelocity       The target Y (toward the left side of the robot) velocity in meters per second.
     * @param angularVelocity The target angular (counter-clockwise positive) velocity in radians per second.
     */
    default void driveRobotOriented(double xVelocity, double yVelocity, double angularVelocity) {
    }

    /**
     * Drives field-oriented at a specified angle with the ability to specify an X, Y velocity.
     *
     * @param xVelocity The target X (downfield) velocity in meters per second.
     * @param yVelocity The target Y (toward the left side of the field) velocity in meters per second.
     * @param angle     The target angle to face while driving.
     */
    default void driveFieldOrientedFacingAngle(double xVelocity, double yVelocity, Rotation2d angle) {
    }

    /**
     * Stops the drivetrain.
     */
    default void stop() {
    }

    default void resetPose(Pose2d pose) {
    }

    default void zeroGyroscope() {
    }

    default void addVisionMeasurement(double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {
    }

    @AutoLog
    class DrivetrainInputs {
        public int successfulDaqs = 0;
        public int failedDaqs = 0;
        public double odometryPeriod = 0;

        public Pose2d pose = new Pose2d();
        public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
        public SwerveModuleState[] moduleStates = new SwerveModuleState[0];
        public SwerveModuleState[] moduleTargets = new SwerveModuleState[0];

        public Rotation3d rotation = new Rotation3d();
    }
}
