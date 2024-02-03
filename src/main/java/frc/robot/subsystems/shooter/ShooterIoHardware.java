package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.AnalogInput;

public class ShooterIoHardware implements ShooterIo {
    private static final double GEAR_RATIO = 1.0;
    private static final double VELOCITY_COEFFICIENT = (2 * Math.PI) / (2048 * GEAR_RATIO) * 10;

    private final TalonFX motor;
    private final AnalogInput beamBreak;
    private Object TalonFXControlMode;

    public ShooterIoHardware(TalonFX motor, AnalogInput beamBreak) {
        this.motor = motor;
        this.beamBreak = beamBreak;
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.angularVelocity = motor.getRotorVelocity() * VELOCITY_COEFFICIENT;
        inputs.currentDrawAmps = motor.getSupplyCurrent();
        inputs.appliedVolts = motor.getMotorVoltage();

        inputs.motorTemp = motor.getExpiration();

        inputs.beamBreakVoltage = beamBreak.getVoltage();
    }

    @Override
    public void setMotorVoltage(double voltage) { motor.set(voltage / 12); }
}
