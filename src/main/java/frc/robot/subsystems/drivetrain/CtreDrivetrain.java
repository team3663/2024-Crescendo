package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;

public class CtreDrivetrain implements DrivetrainIO {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private final SwerveDrivetrain drivetrain;

    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric();
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    public CtreDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... moduleConstants) {
        drivetrain = new SwerveDrivetrain(drivetrainConstants, moduleConstants);
        if (Utils.isSimulation()) {
            startSimThread();
        }
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
