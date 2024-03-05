package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public class CommandFactory {
    private final Climber climber;
    private final Drivetrain drivetrain;
    private final Feeder feeder;
    private final Intake intake;
    private final Led led;
    private final Pivot pivot;
    private final Shooter shooter;
    private final Vision vision;

    public CommandFactory(
            Climber climber,
            Drivetrain drivetrain,
            Feeder feeder,
            Intake intake,
            Led led,
            Pivot pivot,
            Shooter shooter,
            Vision vision
    ) {
        this.climber = climber;
        this.drivetrain = drivetrain;
        this.feeder = feeder;
        this.intake = intake;
        this.led = led;
        this.pivot = pivot;
        this.shooter = shooter;
        this.vision = vision;
    }

    /**
     * Intakes a note from the ground and loads it into the feeder.
     * <p>
     * Uses:
     * <ul>
     *     <li>Feeder</li>
     *     <li>Intake</li>
     * </ul>
     * Finish condition: When a note has been loaded into the feeder.
     */
    public Command intakeAndLoad() {
        // Spin both the feeder and the intake until we detect a piece in the feeder
        return Commands.parallel(
                Commands.waitSeconds(0.25).andThen(Commands.waitUntil(feeder::isDetected))
                        .deadlineWith(
                                intake.runWithVoltage(4.0)),
                feeder.runWithVoltage(6.0)
                        .until(feeder::isDetected)
        );
    }

    public Command shoot() {
        return Commands.deadline(
                Commands.sequence(
                        Commands.waitUntil(shooter::atTargetVelocity),
                        feeder.runWithVoltage(8.0)
                ).until(feeder::isNotDetected),
                shooter.setTargetVelocity(Units.rotationsPerMinuteToRadiansPerSecond(2500))
        );
    }

    public Command level() {
        double[] x = new double[1];
        final double LEVEL_CONSTANT = 0.5;
        return Commands.parallel(
                        Commands.run(() -> x[0] = x[0] + LEVEL_CONSTANT * drivetrain.getRotation().getX() * Robot.defaultPeriodSecs),
                        climber.follow(() -> 0 + Math.max(0.0, x[0]),
                                () -> 0 - Math.min(0.0, x[0])))
                .beforeStarting(() -> x[0] = 0.0);
    }
}
