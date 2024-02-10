package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;


public class C2024IntakeIo implements IntakeIo {
    public final TalonFX intakeMotor;
    public final TalonSRX centeringMotor;

    public C2024IntakeIo(TalonFX intakeMotor, TalonSRX centeringMotor) {
        this.intakeMotor = intakeMotor;
        this.centeringMotor = centeringMotor;

        TalonFXConfiguration fxConfig = new TalonFXConfiguration();
        fxConfig.CurrentLimits.SupplyCurrentLimit = 15.0;
        intakeMotor.getConfigurator().apply(fxConfig);

        TalonSRXConfiguration srxConfig = new TalonSRXConfiguration();
        srxConfig.peakCurrentLimit = 15;
        centeringMotor.configAllSettings(srxConfig);
    }

    @Override
    public void updateInputs(IntakeIo.Inputs inputs) {
        inputs.angularVelocity = intakeMotor.getRotorVelocity().getValueAsDouble();
        inputs.currentDrawAmps = intakeMotor.getSupplyCurrent().getValueAsDouble();
        inputs.appliedVolts = intakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.motorTemp = intakeMotor.getExpiration();
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        intakeMotor.set(voltage);
    }
    @Override
    public void setCenteringVoltage(double voltage) {
        double output = voltage != 0 ? 12 / voltage : 0;
        centeringMotor.set(TalonSRXControlMode.PercentOutput, output);
    }


}
