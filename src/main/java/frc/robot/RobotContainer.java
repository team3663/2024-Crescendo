// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
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

import static frc.robot.Constants.DRIVER_CONTROLLER_PORT;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Pivot pivot;
    private final Led led;
    private final Shooter shooter;
    private final Feeder feeder;
    private final Climber climber;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController =
            new CommandXboxController(DRIVER_CONTROLLER_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(RobotFactory robotFactory) {
        drivetrain = new Drivetrain(robotFactory.createDrivetrainIO());
        intake = new Intake(robotFactory.createIntakeIo());
        pivot = new Pivot(robotFactory.createPivotIo());
        led = new Led(robotFactory.createLedIo());
        shooter = new Shooter(robotFactory.createShooterIo());
        feeder = new Feeder(robotFactory.createFeederIo());
        climber = new Climber(robotFactory.createClimberIo());

        drivetrain.setDefaultCommand(
                drivetrain.drive(
                        () -> -driverController.getLeftY() * drivetrain.getConstants().maxTranslationalVelocity(),
                        () -> -driverController.getLeftX() * drivetrain.getConstants().maxTranslationalVelocity(),
                        () -> -driverController.getRightX() * drivetrain.getConstants().maxRotationalVelocity()
                )
        );

        // Configure the trigger bindings
        configureBindings();
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
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
       return AutoBuilder.buildAuto("New Auto");
    }
}
