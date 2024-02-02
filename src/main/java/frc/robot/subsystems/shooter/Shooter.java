package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIO.Inputs inputs = new ShooterIO.Inputs();

    public Shooter(ShooterIO io) {
        this.io = io;
    }
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        super.setDefaultCommand(defaultCommand);
    }

    public Command shootWithVoltage(double voltage) {
        return runEnd(
                () -> io.setVoltage(voltage),
                () -> io.setVoltage(0.0)
        );
    }

    public Command shootWithAngle(double angle) {
        return runEnd(
                () -> io.setAngle(angle),
                () -> io.setAngle(0.0)
        );
    }

    public Command shootIntoSpeaker(double voltage) {
        return runEnd(
                () -> io.setVoltage(voltage),
                () -> io.setVoltage(0.0)
        );
    }

    public Command shootIntoAmp(double voltage) {
        return runEnd(
                () -> io.setVoltage(voltage),
                () -> io.setVoltage(0.0)
        );
    }

}
