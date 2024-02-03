package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    private final ClimberIo io;
    private final ClimberIo.Inputs inputs = new ClimberIo.Inputs();

    public Climber(ClimberIo io) {
        this.io = io;
    }

    @Override
    public void periodic() {

        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }
}