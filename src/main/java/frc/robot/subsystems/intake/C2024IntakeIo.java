package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;


public class C2024IntakeIo implements IntakeIo {
    private final TalonFX rollerMotor;
    private final TalonFX leftCenteringMotor;
    private final TalonFX rightCenteringMotor;
    private final DigitalInput beamBreak;

    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public C2024IntakeIo(TalonFX rollerMotor, TalonFX leftCenteringMotor, TalonFX rightCenteringMotor,
                         DigitalInput beamBreak) {
        this.rollerMotor = rollerMotor;
        this.leftCenteringMotor = leftCenteringMotor;
        this.rightCenteringMotor = rightCenteringMotor;
        this.beamBreak = beamBreak;

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.CurrentLimits.SupplyCurrentLimit = 15.0;
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerMotor.getConfigurator().apply(rollerConfig);

        TalonFXConfiguration centeringConfig = new TalonFXConfiguration();
        centeringConfig.CurrentLimits.SupplyCurrentLimit = 15;

        leftCenteringMotor.getConfigurator().apply(centeringConfig);

        // Configuration overrides for right
        centeringConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightCenteringMotor.getConfigurator().apply(centeringConfig);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.rollerAngularVelocity = rollerMotor.getVelocity().getValueAsDouble();
        inputs.rollerCurrentDrawAmps = rollerMotor.getSupplyCurrent().getValueAsDouble();
        inputs.rollerAppliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
        inputs.rollerMotorTemp = rollerMotor.getDeviceTemp().getValueAsDouble();

        inputs.leftCenteringAngularVelocity = leftCenteringMotor.getVelocity().getValueAsDouble();
        inputs.leftCenteringCurrentDrawAmps = leftCenteringMotor.getSupplyCurrent().getValueAsDouble();
        inputs.leftCenteringAppliedVolts = leftCenteringMotor.getMotorVoltage().getValueAsDouble();
        inputs.leftCenteringMotorTemp = leftCenteringMotor.getDeviceTemp().getValueAsDouble();

        inputs.rightCenteringAngularVelocity = rightCenteringMotor.getVelocity().getValueAsDouble();
        inputs.rightCenteringCurrentDrawAmps = rightCenteringMotor.getSupplyCurrent().getValueAsDouble();
        inputs.rightCenteringAppliedVolts = rightCenteringMotor.getMotorVoltage().getValueAsDouble();
        inputs.rightCenteringMotorTemp = rightCenteringMotor.getDeviceTemp().getValueAsDouble();

        inputs.beamBreakSignaled = beamBreak.get();
    }

    @Override
    public void setRollerVoltage(double voltage) {
        rollerMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setCenteringVoltage(double voltage) {
        leftCenteringMotor.setControl(voltageRequest.withOutput(voltage));
        rightCenteringMotor.setControl(voltageRequest.withOutput(voltage));
    }
}
