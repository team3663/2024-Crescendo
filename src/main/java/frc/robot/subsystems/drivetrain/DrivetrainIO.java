package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface DrivetrainIO {
    void updateInputs(Inputs inputs);

    /**
     * Drives using a WPILib chassis speeds.
     *
     * @param chassisSpeeds The chassis speeds to follow.
     */
    void drive(ChassisSpeeds chassisSpeeds);

    /**
     * Drives field-oriented with the ability to specify an X, Y, and rotational velocity.
     *
     * @param xVelocity          The target X (downfield) velocity in meters per second.
     * @param yVelocity          The target Y (toward the left side of the field) velocity in meters per second.
     * @param rotationalVelocity The target rotational (counter-clockwise positive) velocity in radians per second.
     */
    void driveFieldOriented(double xVelocity, double yVelocity, double rotationalVelocity);

    /**
     * Drives field-oriented at a specified angle with the ability to specify an X, Y velocity.
     *
     * @param xVelocity The target X (downfield) velocity in meters per second.
     * @param yVelocity The target Y (toward the left side of the field) velocity in meters per second.
     * @param angle     The target angle to face while driving.
     */
    void driveFieldOrientedFacingAngle(double xVelocity, double yVelocity, Rotation2d angle);

    /**
     * Stops the drivetrain.
     */
    void stop();
    HolonomicPathFollowerConfig getPathFollowerConfig();

    void resetPose(Pose2d pose);
    void zeroGyroscope();

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
