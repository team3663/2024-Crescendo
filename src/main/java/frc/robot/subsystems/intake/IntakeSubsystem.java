package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;


public class IntakeSubsystem extends SubsystemBase {
    private final double MAX_VOLTS = 12;
    private final IntakeIO io;
    private final IntakeIO.Inputs inputs = new IntakeIO.Inputs();
    private double intakeVoltage = 0;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Intake", inputs);
    }

}