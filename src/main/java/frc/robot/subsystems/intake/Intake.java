package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final frc.robot.subsystems.intake.IntakeIo io;
    private final IntakeIo.Inputs inputs = new IntakeIo.Inputs();

    public Intake(IntakeIo io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public Command runWithVoltage(double voltage) {
        return runEnd(
                () -> {io.setIntakeVoltage(voltage); io.setCenteringVoltage(voltage);},
                () -> {io.setIntakeVoltage(0.0); io.setCenteringVoltage(0.0);}
        );
    }

}