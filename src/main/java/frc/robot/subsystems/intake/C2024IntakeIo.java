package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;


public class C2024IntakeIo implements IntakeIo {
    public final TalonFX rollerMotor;
    public final TalonSRX centeringMotor;

    public C2024IntakeIo(TalonFX rollerMotor, TalonSRX centeringMotor) {
        this.rollerMotor = rollerMotor;
        this.centeringMotor = centeringMotor;

        TalonFXConfiguration fxConfig = new TalonFXConfiguration();
        fxConfig.CurrentLimits.SupplyCurrentLimit = 15.0;
        rollerMotor.getConfigurator().apply(fxConfig);

        TalonSRXConfiguration srxConfig = new TalonSRXConfiguration();
        srxConfig.peakCurrentLimit = 15;
        centeringMotor.configAllSettings(srxConfig);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.rollerAngularVelocity = rollerMotor.getRotorVelocity().getValueAsDouble();
        inputs.rollerCurrentDrawAmps = rollerMotor.getSupplyCurrent().getValueAsDouble();
        inputs.rollerAppliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
        inputs.rollerMotorTemp = rollerMotor.getDeviceTemp().getValueAsDouble();

        inputs.centeringAngularVelocity = centeringMotor.getActiveTrajectoryVelocity();
        inputs.centeringCurrentDrawAmps = centeringMotor.getSupplyCurrent();
        inputs.centeringAppliedVolts = centeringMotor.getMotorOutputVoltage();
        inputs.centeringMotorTemp = centeringMotor.getTemperature();
    }

    @Override
    public void setRollerVoltage(double voltage) {
        rollerMotor.set(voltage);
    }
    @Override
    public void setCenteringVoltage(double voltage) {
        double output = voltage != 0 ? 12 / voltage : 0;
        centeringMotor.set(TalonSRXControlMode.PercentOutput, output);
    }
}
