package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class C2024ShooterIo implements ShooterIo {
    private static final double GEAR_RATIO = 1.0;
    private static final double VELOCITY_COEFFICIENT = (2 * Math.PI) / (2048 * GEAR_RATIO) * 10;

    private final TalonFX upperMotor;
    private final TalonFX lowerMotor;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);


    public C2024ShooterIo(TalonFX upperMotor, TalonFX lowerMotor) {
        this.upperMotor = upperMotor;
        this.lowerMotor = lowerMotor;

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 1.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        upperMotor.getConfigurator().apply(config);
        lowerMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.angularVelocity = upperMotor.getRotorVelocity().getValueAsDouble() * VELOCITY_COEFFICIENT;
        inputs.currentDrawAmps = upperMotor.getSupplyCurrent().getValueAsDouble();
        inputs.appliedVolts = upperMotor.getMotorVoltage().getValueAsDouble();
        inputs.motorTemp = upperMotor.getExpiration();

        inputs.angularVelocity = lowerMotor.getRotorVelocity().getValueAsDouble() * VELOCITY_COEFFICIENT;
        inputs.currentDrawAmps = lowerMotor.getSupplyCurrent().getValueAsDouble();
        inputs.appliedVolts = lowerMotor.getMotorVoltage().getValueAsDouble();
        inputs.motorTemp = lowerMotor.getExpiration();

    }

    @Override
    public void setTargetVelocity(double velocity) {
        upperMotor.setControl(velocityRequest.withVelocity(velocity));
        lowerMotor.setControl(velocityRequest.withVelocity(velocity));


    }
}