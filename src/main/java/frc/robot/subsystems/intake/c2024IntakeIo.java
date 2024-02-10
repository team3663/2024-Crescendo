package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class c2024IntakeIo implements IntakeIo {
    public final TalonFX motor;

    public c2024IntakeIo(TalonFX motor) {
        this.motor = motor;

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Voltage.PeakForwardVoltage = 0.0;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(IntakeIo.Inputs inputs) {
        inputs.angularVelocity = motor.getRotorVelocity().getValueAsDouble();
        inputs.currentDrawAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.motorTemp = motor.getExpiration();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.set(voltage);
    }

}
