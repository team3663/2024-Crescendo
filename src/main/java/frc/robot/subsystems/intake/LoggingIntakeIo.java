package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

public class LoggingIntakeIo implements IntakeIo {
    private final IntakeIo inner;

    public LoggingIntakeIo(IntakeIo inner) {
        this.inner = inner;
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inner.updateInputs(inputs);
    }

    @Override
    public void setRollerVoltage(double voltage) {
        Logger.recordOutput("Intake/TargetRollerVoltage", voltage);

        inner.setRollerVoltage(voltage);
    }

    @Override
    public void setCenteringVoltage(double voltage) {
        Logger.recordOutput("Intake/TargetCenteringVoltage", voltage);

        inner.setCenteringVoltage(voltage);
    }
}
