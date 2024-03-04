package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotFactory;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.LedColor;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.ControllerHelper;
import frc.robot.utility.RobotMode;

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

        drivetrain.setDefaultCommand(
                drivetrain.drive(
                        () -> -ControllerHelper.modifyAxis(driverController.getLeftY()) * drivetrain.getConstants().maxTranslationalVelocity(),
                        () -> -ControllerHelper.modifyAxis(driverController.getLeftX()) * drivetrain.getConstants().maxTranslationalVelocity(),
                        () -> -ControllerHelper.modifyAxis(driverController.getRightX()) * drivetrain.getConstants().maxRotationalVelocity()
                )
        );

        // Periodically adds the vision measurement to drivetrain for pose estimation
        vision.setDefaultCommand(
                vision.updateVisionMeasurements(drivetrain)
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
                .whileTrue(commandFactory.intakeAndLoad());
        driverController.rightTrigger()
                .whileTrue(commandFactory.shoot());
//        driverController.rightBumper()
//                .whileTrue(shooter.setTargetVelocity(rotationsPerMinuteToRadiansPerSecond(2500)));
        // AMP: 1500 RPM, 120 deg, 0.5 in from wall
        driverController.rightBumper()
                .onTrue(pivot.moveTo(Units.degreesToRadians(50.0)))
                .onFalse(pivot.moveTo(Units.degreesToRadians(2.0)));

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
        driverController.b().onTrue(RobotMode.scoreLocation(RobotMode.ScoreLocation.TRAP));
    }

    private void configureTestBinding() {
        testController.a().onTrue(new InstantCommand(() -> led.setColor(new LedColor(0, 255, 0))));
        testController.b().onTrue(new InstantCommand(() -> led.setColor(new LedColor(255, 0, 0))));
        testController.x().onTrue(new InstantCommand(() -> led.setColor(new LedColor(0, 0, 255))));
        testController.y().onTrue(new InstantCommand(() -> led.setColor(new LedColor(0, 0, 0))));

        testController.povUp().onTrue(new InstantCommand(() -> led.setPattern(Led.Pattern.SOLID)));
        testController.povDown().onTrue(new InstantCommand(() -> led.setPattern(Led.Pattern.STROBE)));
        testController.povRight().onTrue(new InstantCommand(() -> led.setPattern(Led.Pattern.LARSON)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }
}
