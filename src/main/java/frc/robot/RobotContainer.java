// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.RobotFactory;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

import static edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond;
import static frc.robot.Constants.DRIVER_CONTROLLER_PORT;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
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
                        () -> -driverController.getLeftY() * drivetrain.getConstants().maxTranslationalVelocity(),
                        () -> -driverController.getLeftX() * drivetrain.getConstants().maxTranslationalVelocity(),
                        () -> -driverController.getRightX() * drivetrain.getConstants().maxRotationalVelocity()
                )
        );

        NamedCommands.registerCommand( "shootNote", shooter.shootCommand();
        NamedCommands.registerCommand("intakeNote", intake.intakeCommand();

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

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        driverController.start()
                .onTrue(drivetrain.zeroGyroscope());

        driverController.a()
                .whileTrue(shooter.setTargetVelocity(rotationsPerMinuteToRadiansPerSecond(3000)))
                .onFalse(shooter.stop());

        driverController.leftTrigger()
                .whileTrue(commandFactory.intakeAndLoad());
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
