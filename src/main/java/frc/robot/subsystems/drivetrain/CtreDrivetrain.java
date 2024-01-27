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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class CtreDrivetrain implements DrivetrainIO {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private final SwerveDrivetrain drivetrain;

    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private double maxModuleSpeed;
    private double driveBaseRadius;
    private SwerveDriveKinematics kinematics;
    private HolonomicPathFollowerConfig pathFollowerConfig;
    private final SwerveRequest.ApplyChassisSpeeds applyChassisSpeedsRequest = new SwerveRequest.ApplyChassisSpeeds();
    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric();
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    public CtreDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... moduleConstants) {
        drivetrain = new SwerveDrivetrain(drivetrainConstants, moduleConstants);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        Translation2d[] modulePositions = new Translation2d[moduleConstants.length];
        maxModuleSpeed = moduleConstants[0].SpeedAt12VoltsMps;
        for(int input = 0; input < moduleConstants.length; input++){
            double x = moduleConstants[input].LocationX;
            double y = moduleConstants[input].LocationY;
            modulePositions[input] = new Translation2d(x,y);
            double moduleSpeed = moduleConstants[input].SpeedAt12VoltsMps;
            maxModuleSpeed = Math.min(moduleSpeed,maxModuleSpeed);
            double driveBase = Math.hypot(x,y);
            driveBaseRadius= Math.max(driveBase, driveBaseRadius);
        }

        pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0),
                maxModuleSpeed,
                driveBaseRadius,
                new ReplanningConfig()
        );
        kinematics= new SwerveDriveKinematics(modulePositions);
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
    public void stop() {
        drivetrain.setControl(idleRequest);
    }

    @Override
    public HolonomicPathFollowerConfig getPathFollowerConfig() {
        return pathFollowerConfig;
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
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            drivetrain.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
