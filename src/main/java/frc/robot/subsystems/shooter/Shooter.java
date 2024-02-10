package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIo io;
    private final ShooterIo.Inputs inputs = new ShooterIo.Inputs();

    public Shooter(ShooterIo io) {
        this.io = io;
    }
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public Command shootWithVoltage(double voltage) {
        return runEnd(
                () -> io.setVoltage(voltage),
                () -> io.setVoltage(0.0)
        );
    }

}
