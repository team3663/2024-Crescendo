package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.DRIVETRAIN_TRACKWIDTH;
import static frc.robot.Constants.DRIVETRAIN_WHEELBASE;

public class Drivetrain extends SubsystemBase {
    private static final String[] MODULE_NAMES = {
            "Front Left", "Front Right", "Back Left", "Back Right"
    };
    private final GyroscopeIO gyroscope;
    private final GyroscopeIO.Inputs gyroscopeInputs = new GyroscopeIO.Inputs();
    private final SwerveModuleIO[] modules;
    private final SwerveModuleIO.Inputs[] moduleInputs;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();

    public Drivetrain(
            GyroscopeIO gyroscope,
            SwerveModuleIO frontLeftModule,
            SwerveModuleIO frontRightModule,
            SwerveModuleIO backLeftModule,
            SwerveModuleIO backRightModule) {
        this.gyroscope = gyroscope;
        modules = new SwerveModuleIO[]{frontLeftModule, frontRightModule, backLeftModule, backRightModule};
        moduleInputs = new SwerveModuleIO.Inputs[modules.length];
        for (int i = 0; i < moduleInputs.length; i++)
            moduleInputs[i] = new SwerveModuleIO.Inputs();

        kinematics = new SwerveDriveKinematics(
                new Translation2d(DRIVETRAIN_WHEELBASE / 2.0, DRIVETRAIN_TRACKWIDTH / 2.0),
                new Translation2d(DRIVETRAIN_WHEELBASE / 2.0, -DRIVETRAIN_TRACKWIDTH / 2.0),
                new Translation2d(-DRIVETRAIN_WHEELBASE / 2.0, DRIVETRAIN_TRACKWIDTH / 2.0),
                new Translation2d(-DRIVETRAIN_WHEELBASE / 2.0, -DRIVETRAIN_TRACKWIDTH / 2.0)
        );

        SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++)
            modulePositions[i] = new SwerveModulePosition();
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), modulePositions);
    }

    @Override
    public void periodic() {
        gyroscope.updateInputs(gyroscopeInputs);
        Logger.processInputs("Drivetrain/Gyroscope", gyroscopeInputs);

        SwerveModuleState[] currentModuleStates = new SwerveModuleState[modules.length];
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            SwerveModuleIO module = modules[i];
            SwerveModuleIO.Inputs moduleInput = moduleInputs[i];

            module.updateInputs(moduleInput);
            Logger.processInputs("Drivetrain/" + MODULE_NAMES[i], moduleInput);

            currentModuleStates[i] = new SwerveModuleState(0.0, Rotation2d.fromRadians(moduleInput.steerAngleRad));
            modulePositions[i] = new SwerveModulePosition(0.0, Rotation2d.fromRadians(moduleInput.steerAngleRad));
        }
        odometry.update(gyroscopeInputs.yaw, modulePositions);

        SwerveModuleState[] targetModuleStates = kinematics.toSwerveModuleStates(targetChassisSpeeds);
        for (int i = 0; i < modules.length; i++)
            modules[i].setTargetState(targetModuleStates[i]);

        Logger.recordOutput("Drivetrain/CurrentPose", odometry.getPoseMeters());
        Logger.recordOutput("Drivetrain/CurrentModuleStates", currentModuleStates);
        Logger.recordOutput("Drivetrain/TargetModuleStates", targetModuleStates);
        Logger.recordOutput("Drivetrain/TargetChassisSpeeds", targetChassisSpeeds);
    }

    public Command drive(
            DoubleSupplier xVelocity,
            DoubleSupplier yVelocity,
            DoubleSupplier rotationalVelocity
    ) {
        return runEnd(
                // execute()
                () -> targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xVelocity.getAsDouble(),
                        yVelocity.getAsDouble(),
                        rotationalVelocity.getAsDouble(),
                        odometry.getPoseMeters().getRotation()
                ),
                // end()
                () -> targetChassisSpeeds = new ChassisSpeeds()
        );
    }

    public Command zeroGyroscope() {
        return Commands.runOnce(() -> {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
            for (int i = 0; i < modules.length; i++)
                modulePositions[i] = new SwerveModulePosition(0, Rotation2d.fromRadians(moduleInputs[i].steerAngleRad));

            odometry.resetPosition(
                    gyroscopeInputs.yaw,
                    modulePositions,
                    new Pose2d(odometry.getPoseMeters().getTranslation(), new Rotation2d())
            );
        });
    }
}
