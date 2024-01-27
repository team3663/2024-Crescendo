package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface DrivetrainIO {
    void updateInputs(Inputs inputs);

    void drive(ChassisSpeeds chassisSpeeds);

    void driveFieldOriented(double xVelocity, double yVelocity, double rotationalVelocity);

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
