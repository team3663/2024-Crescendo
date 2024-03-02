package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class C2024ShooterIo implements ShooterIo {
    private static final double GEAR_RATIO = 1.0;
    private static final double VELOCITY_COEFFICIENT = (2 * Math.PI) / (2048 * GEAR_RATIO) * 10;

    private final TalonFX upperMotor;
    private final TalonFX lowerMotor;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final CoastOut stopRequest = new CoastOut();


    public C2024ShooterIo(TalonFX upperMotor, TalonFX lowerMotor) {
        this.upperMotor = upperMotor;
        this.lowerMotor = lowerMotor;

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 1.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        upperMotor.getConfigurator().apply(config);

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        lowerMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        inputs.upperAngularVelocity = upperMotor.getRotorVelocity().getValueAsDouble() * VELOCITY_COEFFICIENT;
        inputs.upperCurrentDrawAmps = upperMotor.getSupplyCurrent().getValueAsDouble();
        inputs.upperAppliedVolts = upperMotor.getMotorVoltage().getValueAsDouble();
        inputs.upperMotorTemp = upperMotor.getDeviceTemp().getValueAsDouble();

        inputs.lowerAngularVelocity = lowerMotor.getRotorVelocity().getValueAsDouble() * VELOCITY_COEFFICIENT;
        inputs.lowerCurrentDrawAmps = lowerMotor.getSupplyCurrent().getValueAsDouble();
        inputs.lowerAppliedVolts = lowerMotor.getMotorVoltage().getValueAsDouble();
        inputs.lowerMotorTemp = lowerMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setTargetVelocity(double velocity) {
        upperMotor.setControl(velocityRequest.withVelocity(velocity));
        lowerMotor.setControl(velocityRequest.withVelocity(velocity));
    }

    @Override
    public void setVoltage(double voltage) {
        upperMotor.setControl(voltageRequest.withOutput(voltage));
        lowerMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void stop() {
        upperMotor.setControl(stopRequest);
        lowerMotor.setControl(stopRequest);
    }
}
