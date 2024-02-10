package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.AnalogInput;

public class c2024ShooterIo implements ShooterIo {
    private static final double GEAR_RATIO = 1.0;
    private static final double VELOCITY_COEFFICIENT = (2 * Math.PI) / (2048 * GEAR_RATIO) * 10;

    private final TalonFX primaryMotor;
    private final TalonFX secondaryMotor;
    private final AnalogInput beamBreak;


    public c2024ShooterIo(TalonFX primaryMotor, TalonFX secondaryMotor, AnalogInput beamBreak) {
        this.primaryMotor = primaryMotor;
        this.secondaryMotor = secondaryMotor;
        this.beamBreak = beamBreak;

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 5.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        primaryMotor.getConfigurator().apply(config);
        secondaryMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.angularVelocity = primaryMotor.getRotorVelocity().getValueAsDouble() * VELOCITY_COEFFICIENT;
        inputs.currentDrawAmps = primaryMotor.getSupplyCurrent().getValueAsDouble();
        inputs.appliedVolts = primaryMotor.getMotorVoltage().getValueAsDouble();
        inputs.motorTemp = primaryMotor.getExpiration();

        inputs.angularVelocity = secondaryMotor.getRotorVelocity().getValueAsDouble() * VELOCITY_COEFFICIENT;
        inputs.currentDrawAmps = secondaryMotor.getSupplyCurrent().getValueAsDouble();
        inputs.appliedVolts = secondaryMotor.getMotorVoltage().getValueAsDouble();
        inputs.motorTemp = secondaryMotor.getExpiration();

        inputs.beamBreakVoltage = beamBreak.getVoltage();
    }

    @Override
    public void setPrimaryMotorVoltage(double voltage) { primaryMotor.set(voltage / 12); }

    @Override
    public void setSecondaryMotorVoltage(double voltage) { secondaryMotor.set(voltage / 12); }
}
