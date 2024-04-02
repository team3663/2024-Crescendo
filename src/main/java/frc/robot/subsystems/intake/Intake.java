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

    public boolean isDetected() {
        return inputs.beamBreakSignaled;
    }

    public Command withVoltage(double voltage) {
        return withVoltage(voltage, voltage);
    }

    public Command withVoltage(double rollerVoltage, double centeringVoltage) {
        return runEnd(
                () -> {
                    io.setRollerVoltage(rollerVoltage);
                    io.setCenteringVoltage(centeringVoltage);
                },
                () -> {
                    io.setRollerVoltage(0.0);
                    io.setCenteringVoltage(0.0);
                }
        );
    }

}