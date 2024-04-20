package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotFactory;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.ControllerHelper;
import frc.robot.utility.RobotMode;
import org.littletonrobotics.junction.Logger;

import java.util.EnumMap;
import java.util.Map;

import static frc.robot.Constants.*;

public class RobotContainer {
    private final Climber climber;
    private final Drivetrain drivetrain;
    private final Feeder feeder;
    private final Intake intake;
    private final Led led;
    private final Pivot pivot;
    private final Shooter shooter;
    private final Vision vision;

    private final CommandFactory commandFactory;

    private final CommandXboxController driverController =
            new CommandXboxController(DRIVER_CONTROLLER_PORT);

    private final CommandXboxController testController;

    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer(RobotFactory robotFactory) {
        climber = new Climber(robotFactory.createClimberIo());
        drivetrain = new Drivetrain(robotFactory.createDrivetrainIO());
        feeder = new Feeder(robotFactory.createFeederIo());
        intake = new Intake(robotFactory.createIntakeIo());
        led = new Led(robotFactory.createLedIo());
        pivot = new Pivot(robotFactory.createPivotIo());
        shooter = new Shooter(robotFactory.createShooterIo());
        vision = new Vision(robotFactory.createVisionIo());

        commandFactory = new CommandFactory(climber, drivetrain, feeder, intake, led, pivot, shooter, vision);

        drivetrain.setDefaultCommand(drivetrain.drive(this::getDrivetrainXVelocity, this::getDrivetrainYVelocity, this::getDrivetrainAngularVelocity));

        // When nothing is happening, return to the resting angle
        pivot.setDefaultCommand(pivot.follow(() -> pivot.getConstants().restingPivotAngle()));

        led.setDefaultCommand(led.signalCommand(intake::isDetected, feeder::isDetected, driverController.povLeft())
                .ignoringDisable(true));

        // Periodically adds the vision measurement to drivetrain for pose estimation
        vision.setDefaultCommand(
                vision.consumeVisionMeasurements(drivetrain::addVisionMeasurements).ignoringDisable(true)
        );
        // Configure controller binding.
        configureBindings();

        // If test features are turned on then create the test controller object and bind it.
        if (ENABLE_TEST_FEATURES) {
            testController = new CommandXboxController(TEST_CONTROLLER_PORT);
            configureTestBinding();
        } else {
            testController = null;
        }

        NamedCommands.registerCommand("shootNote",
                Commands.sequence(
                        Commands.either(
                                commandFactory.intakeAndLoad().withTimeout(4.0),
                                Commands.none(),
                                intake::isDetected
                        ),
                        Commands.either(
                                commandFactory.aimAndFireAtSpeaker(() -> true, () -> 0.0, () -> 0.0, () -> 0.0)
                                        .andThen(pivot.moveTo(pivot.getConstants().restingPivotAngle())),
                                Commands.none(),
                                feeder::isDetected)
                ));
        NamedCommands.registerCommand("shootSubwooferNote",
                commandFactory.aimAndFireAtSubwoofer(() -> true, () -> 0.0, () -> 0.0, () -> 0.0)
                        .andThen(pivot.moveTo(pivot.getConstants().restingPivotAngle())));
        NamedCommands.registerCommand("intakeNote",
                commandFactory.intakeAndLoad());
        NamedCommands.registerCommand("intakeQuick",
                commandFactory.intakeQuick());
        NamedCommands.registerCommand("waitForIntake",
                Commands.either(
                        Commands.waitUntil(feeder::isDetected).withTimeout(2),
                        Commands.waitUntil(intake::isDetected).withTimeout(0.25)
                                .andThen(Commands.waitUntil(feeder::isDetected)
                                        .withTimeout(2)
                                        .onlyIf(intake::isDetected)),
                        intake::isDetected
                ));
        NamedCommands.registerCommand("zero", pivot.zero());

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Driver")
                .add("Auto Chooser", autoChooser)
                .withPosition(0, 0)
                .withSize(3, 1)
                .withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    private void configureBindings() {
        driverController.back()
                .onTrue(drivetrain.zeroGyroscope());

        driverController.leftTrigger()
                .whileTrue(commandFactory.intakeAndLoad()
                        .andThen(ControllerHelper.rumble(driverController)));
        driverController.leftBumper()
                .whileTrue(intake.withVoltage(-4.0, 0.0));

        Map<RobotMode.ScoreLocation, Command> robotModeCommandMap = new EnumMap<>(RobotMode.ScoreLocation.class);
        robotModeCommandMap.put(RobotMode.ScoreLocation.AMP, commandFactory.aimAndFireAtAmp(
                // Allowed to fire when left trigger is held
                driverController.rightTrigger(),
                // Drive using normal controls
                this::getDrivetrainXVelocity, this::getDrivetrainYVelocity, this::getDrivetrainAngularVelocity
        ));
        robotModeCommandMap.put(RobotMode.ScoreLocation.SPEAKER, commandFactory.aimAndFireAtSpeaker(
                // Allowed to fire when left trigger is held
                driverController.rightTrigger(),
                // Drive using normal controls
                this::getDrivetrainXVelocity, this::getDrivetrainYVelocity, this::getDrivetrainAngularVelocity
        ));
        robotModeCommandMap.put(RobotMode.ScoreLocation.SUBWOOFER, commandFactory.aimAndFireAtSubwoofer(
                // Allowed to fire when left trigger is held
                driverController.rightTrigger(),
                // Drive using normal controls
                this::getDrivetrainXVelocity, this::getDrivetrainYVelocity, this::getDrivetrainAngularVelocity
        ));
        robotModeCommandMap.put(RobotMode.ScoreLocation.PASS, commandFactory.aimAndPass(
                // Allowed to fire when left trigger is held
                driverController.rightTrigger(),
                // Drive using normal controls
                this::getDrivetrainXVelocity, this::getDrivetrainYVelocity, this::getDrivetrainAngularVelocity
        ));


        // Don't allow aiming when the intake button is held
        driverController.rightBumper()
                .and(driverController.leftTrigger().negate())
                .whileTrue(Commands.select(robotModeCommandMap, RobotMode::getScoreLocation));

        // Climber controls
        driverController.start()
                .onTrue(Commands.parallel(climber.zero(), pivot.zero()));
        driverController.povUp()
                .onTrue(climber.moveTo(climber.getConstants().maxPosition())
                        .beforeStarting(climber.unlock())
                        .andThen(climber.lock()));
        driverController.povDown()
                .onTrue(climber.unlock().andThen(climber.follow(climber.getConstants()::minPosition)).andThen(climber.lock()));

        // Scoring location controls
        driverController.x().onTrue(RobotMode.scoreLocation(RobotMode.ScoreLocation.AMP));
        driverController.y().onTrue(RobotMode.scoreLocation(RobotMode.ScoreLocation.SPEAKER));
        driverController.a().onTrue(RobotMode.scoreLocation(RobotMode.ScoreLocation.SUBWOOFER));
        driverController.b().onTrue(RobotMode.scoreLocation(RobotMode.ScoreLocation.PASS));

    }

    private void configureTestBinding() {
        // Shooter tuning button
        // A button toggles whether we are in tuning mode
        // POV UP/DOWN increases/decreases the shooter velocity
        // POV RIGHT/LEFT increases/decreases the pivot angle

        final double TUNING_PIVOT_ANGLE_CHANGE = Units.degreesToRadians(0.25);
        final double TUNING_SHOOTER_VELOCITY_CHANGE = Units.rotationsPerMinuteToRadiansPerSecond(50.0);

        double[] tuningPivotAngle = new double[]{pivot.getConstants().minPivotAngle()};
        double[] tuningShooterVelocity = new double[]{0.0};

        testController.a().toggleOnTrue(Commands.parallel(
                pivot.follow(() -> tuningPivotAngle[0]),
                shooter.withVelocity(() -> tuningShooterVelocity[0]),
                Commands.run(() -> {
                    Translation2d speakerPosition = DriverStation.getAlliance()
                            .map(Constants::getSpeakerPositionForAlliance)
                            .orElse(new Translation2d());

                    Translation2d robotPosition = drivetrain.getPose().getTranslation();

                    double distance = speakerPosition.getDistance(robotPosition);

                    Logger.recordOutput("Tuning/Distance", distance);
                    Logger.recordOutput("Tuning/TargetPivotAngle", tuningPivotAngle[0]);
                    Logger.recordOutput("Tuning/TargetShooterVelocity", tuningShooterVelocity[0]);
                })
        ));

        testController.rightTrigger().whileTrue(feeder.runWithVoltage(12.0));

        testController.povUp().onTrue(Commands.runOnce(() -> tuningShooterVelocity[0] += TUNING_SHOOTER_VELOCITY_CHANGE));
        testController.povDown().onTrue(Commands.runOnce(() -> tuningShooterVelocity[0] -= TUNING_SHOOTER_VELOCITY_CHANGE));

        testController.povLeft().onTrue(Commands.runOnce(() -> tuningPivotAngle[0] -= TUNING_PIVOT_ANGLE_CHANGE));
        testController.povRight().onTrue(Commands.runOnce(() -> tuningPivotAngle[0] += TUNING_PIVOT_ANGLE_CHANGE));
    }

    private double getDrivetrainXVelocity() {
        return -ControllerHelper.modifyAxis(driverController.getLeftY()) * drivetrain.getConstants().maxLinearVelocity();
    }

    private double getDrivetrainYVelocity() {
        return -ControllerHelper.modifyAxis(driverController.getLeftX()) * drivetrain.getConstants().maxLinearVelocity();
    }

    private double getDrivetrainAngularVelocity() {
        return -ControllerHelper.modifyAxis(driverController.getRightX()) * drivetrain.getConstants().maxAngularVelocity();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return new ProxyCommand(autoChooser.getSelected())
                // Initially zero the pivot in-place
                // This is instant, and should give us a good enough zero for autonomous
                // This means that the pivot MUST be resting at the bottom of travel
                .beforeStarting(pivot.zeroInPlace())
                .alongWith(climber.zero());
    }
}
