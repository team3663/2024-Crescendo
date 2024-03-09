package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

public class CtreDrivetrain extends SwerveDrivetrain implements DrivetrainIO {
    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private final Drivetrain.Constants constants;

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
        super(drivetrainConstants, 0.0,
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(10.0, 10.0, 10.0),
                moduleConstants);
        fieldCentricFacingAngleRequest.HeadingController.setPID(10.0, 0.0, 0.0);
        fieldCentricFacingAngleRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        fieldCentricFacingAngleRequest.ForwardReference = SwerveRequest.ForwardReference.RedAlliance;

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
    public void updateInputs(DrivetrainInputs inputs) {
        SwerveDrivetrain.SwerveDriveState state = this.getState();

        inputs.successfulDaqs = state.SuccessfulDaqs;
        inputs.failedDaqs = state.FailedDaqs;
        inputs.odometryPeriod = state.OdometryPeriod;

        inputs.pose = state.Pose;
        inputs.moduleStates = state.ModuleStates == null ? new SwerveModuleState[0] : state.ModuleStates;
        inputs.moduleTargets = state.ModuleTargets == null ? new SwerveModuleState[0] : state.ModuleTargets;
        inputs.chassisSpeeds = kinematics.toChassisSpeeds(inputs.moduleStates);
        inputs.rotation = this.getRotation3d();
    }

    @Override
    public void drive(ChassisSpeeds chassisSpeeds) {
        this.setControl(applyChassisSpeedsRequest.withSpeeds(chassisSpeeds));
    }

    @Override
    public void driveFieldOriented(double xVelocity, double yVelocity, double angularVelocity) {
        this.setControl(
                fieldCentricRequest
                        .withVelocityX(xVelocity)
                        .withVelocityY(yVelocity)
                        .withRotationalRate(angularVelocity)
        );
    }

    @Override
    public void driveFieldOrientedFacingAngle(double xVelocity, double yVelocity, Rotation2d angle) {
        this.setControl(
                fieldCentricFacingAngleRequest
                        .withVelocityX(xVelocity)
                        .withVelocityY(yVelocity)
                        .withTargetDirection(angle)
        );
    }

    @Override
    public void stop() {
        this.setControl(idleRequest);
    }

    @Override
    public void resetPose(Pose2d pose) {
        this.seedFieldRelative(pose);
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            Rotation2d forwardDirection = switch (alliance.get()) {
                case Red -> Rotation2d.fromDegrees(180.0);
                case Blue -> Rotation2d.fromDegrees(0.0);
            };

            this.setOperatorPerspectiveForward(forwardDirection);
        }
    }

    @Override
    public void zeroGyroscope() {
        boolean flipOrientation = false;

        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            flipOrientation = alliance.get() == DriverStation.Alliance.Red;
        }

        try {
            m_stateLock.writeLock().lock();

            m_fieldRelativeOffset =
                    flipOrientation ?
                            getState().Pose.getRotation().rotateBy(Rotation2d.fromDegrees(180.0))
                            : getState().Pose.getRotation();
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    @Override
    public void addVisionMeasurement(double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {
        this.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    private void startSimThread() {
        lastSimTime = Timer.getFPGATimestamp();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Timer.getFPGATimestamp();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            this.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }

    public record Constants(
            PIDConstants translationFollowerConstants,
            PIDConstants rotationFollowerConstants
    ) {
    }
}
