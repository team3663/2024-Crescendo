package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIo io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    public Intake(IntakeIo io) {
        this.io = new LoggingIntakeIo(io);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public Command runWithVoltage(double voltage) {
        return runEnd(
                () -> {
                    io.setRollerVoltage(voltage);
                    io.setCenteringVoltage(voltage / 4.0);
                },
                () -> {
                    io.setRollerVoltage(0.0);
                    io.setCenteringVoltage(0.0);
                }
        );
    }

}