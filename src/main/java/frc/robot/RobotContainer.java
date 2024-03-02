package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotFactory;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utility.ControllerHelper;

import static edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond;
import static frc.robot.Constants.DRIVER_CONTROLLER_PORT;


public class RobotContainer {
    private final Climber climber;
    private final Drivetrain drivetrain;
    private final Feeder feeder;
    private final Intake intake;
    private final Led led;
    private final Pivot pivot;
    private final Shooter shooter;

    private final CommandFactory commandFactory;

    private final CommandXboxController driverController =
            new CommandXboxController(DRIVER_CONTROLLER_PORT);

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

        commandFactory = new CommandFactory(climber, drivetrain, feeder, intake, led, pivot, shooter);

        drivetrain.setDefaultCommand(
                drivetrain.drive(
                        () -> -ControllerHelper.modifyAxis(driverController.getLeftY()) * drivetrain.getConstants().maxTranslationalVelocity(),
                        () -> -ControllerHelper.modifyAxis(driverController.getLeftX()) * drivetrain.getConstants().maxTranslationalVelocity(),
                        () -> -ControllerHelper.modifyAxis(driverController.getRightX()) * drivetrain.getConstants().maxRotationalVelocity()
                )
        );

        // Configure the trigger bindings
        configureBindings();

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
                .whileTrue(shooter.runWithVoltage(4.0));
        driverController.rightBumper()
                .whileTrue(shooter.setTargetVelocity(rotationsPerMinuteToRadiansPerSecond(2500)));

        // Climber controls
        driverController.start()
                .onTrue(Commands.parallel(climber.zero(), pivot.zero()));
        driverController.povUp()
                .onTrue(climber.moveTo(climber.getConstants().maxArmHeight())
                        .beforeStarting(climber.unlock())
                        .andThen(climber.lock()));
        driverController.povDown()
                .onTrue(climber.unlock().andThen(commandFactory.level()).andThen(climber.lock()));
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
