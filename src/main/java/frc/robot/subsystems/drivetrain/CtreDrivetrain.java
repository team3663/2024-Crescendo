package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionIo;
import org.photonvision.EstimatedRobotPose;

import java.util.List;

public class CtreDrivetrain implements DrivetrainIO {
    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private final Drivetrain.Constants constants;
    private final SwerveDrivetrain drivetrain;

    private Notifier simNotifier = null;
    private double lastSimTime;
    private SwerveDriveKinematics kinematics;
    private final SwerveRequest.ApplyChassisSpeeds applyChassisSpeedsRequest = new SwerveRequest.ApplyChassisSpeeds();
    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric();
    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle();
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    public CtreDrivetrain(
            Constants constants,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants... moduleConstants) {
        Translation2d[] modulePositions = new Translation2d[moduleConstants.length];
        double maxModuleVelocity = Double.MAX_VALUE;
        double maxDriveBaseRadius = 0.0;
        for (int index = 0; index < moduleConstants.length; index++) {
            double x = moduleConstants[index].LocationX;
            double y = moduleConstants[index].LocationY;
            double moduleVelocity = moduleConstants[index].SpeedAt12VoltsMps;
            double driveBaseRadius = Math.hypot(x, y);

            modulePositions[index] = new Translation2d(x, y);
            maxModuleVelocity = Math.min(maxModuleVelocity, moduleVelocity);
            maxDriveBaseRadius = Math.max(maxDriveBaseRadius, driveBaseRadius);
        }

        this.constants = new Drivetrain.Constants(
                maxModuleVelocity,
                maxDriveBaseRadius,
                new HolonomicPathFollowerConfig(
                        constants.translationFollowerConstants(),
                        constants.rotationFollowerConstants(),
                        maxModuleVelocity,
                        maxDriveBaseRadius,
                        new ReplanningConfig())
        );

        drivetrain = new SwerveDrivetrain(drivetrainConstants, moduleConstants);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        kinematics = new SwerveDriveKinematics(modulePositions);
    }

    @Override
    public Drivetrain.Constants getConstants() {
        return constants;
    }

    @Override
    public void updateInputs(Inputs inputs) {
        SwerveDrivetrain.SwerveDriveState state = drivetrain.getState();

        inputs.successfulDaqs = state.SuccessfulDaqs;
        inputs.failedDaqs = state.FailedDaqs;
        inputs.odometryPeriod = state.OdometryPeriod;

        inputs.pose = state.Pose;
        inputs.moduleStates = state.ModuleStates;
        inputs.moduleTargets = state.ModuleTargets;
        inputs.chassisSpeeds = kinematics.toChassisSpeeds(inputs.moduleStates);
    }

    @Override
    public void drive(ChassisSpeeds chassisSpeeds) {
        drivetrain.setControl(applyChassisSpeedsRequest.withSpeeds(chassisSpeeds));
    }

    @Override
    public void driveFieldOriented(double xVelocity, double yVelocity, double rotationalVelocity) {
        drivetrain.setControl(
                fieldCentricRequest
                        .withVelocityX(xVelocity)
                        .withVelocityY(yVelocity)
                        .withRotationalRate(rotationalVelocity)
        );
    }

    @Override
    public void driveFieldOrientedFacingAngle(double xVelocity, double yVelocity, Rotation2d angle) {
        drivetrain.setControl(
                fieldCentricFacingAngleRequest
                        .withVelocityX(xVelocity)
                        .withVelocityY(yVelocity)
                        .withTargetDirection(angle)
        );
    }

    @Override
    public void stop() {
        drivetrain.setControl(idleRequest);
    }

    @Override
    public void resetPose(Pose2d pose) {
        drivetrain.seedFieldRelative(pose);
    }

    @Override
    public void zeroGyroscope() {
        drivetrain.seedFieldRelative();
    }

    private void startSimThread() {
        lastSimTime = Timer.getFPGATimestamp();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Timer.getFPGATimestamp();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            drivetrain.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }

    public void addVisionMeasurements(List<EstimatedRobotPose> estimatedRobotPoses) {
        for(EstimatedRobotPose estimatedRobotPose : estimatedRobotPoses) {
            drivetrain.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
        }
    }

    public record Constants(
            PIDConstants translationFollowerConstants,
            PIDConstants rotationFollowerConstants
    ) {
    }
}
