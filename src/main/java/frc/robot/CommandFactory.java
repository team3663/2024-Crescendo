package frc.robot;

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
                        intake.runWithVoltage(6.0),
                        feeder.runWithVoltage(4.0)
                ).until(feeder::isDetected)
                // Reverse the intake for a short amount of time and reverse the feeder until no piece is detected
                .andThen(Commands.parallel(
                        intake.runWithVoltage(-3.0).withTimeout(0.25),
                        feeder.runWithVoltage(-1.0).until(() -> !feeder.isDetected())
                ));
    }

    public Command shoot() {
        return null;
    }
}
