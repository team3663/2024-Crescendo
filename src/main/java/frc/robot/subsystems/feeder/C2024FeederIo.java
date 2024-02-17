package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.AnalogInput;

import static edu.wpi.first.math.util.Units.inchesToMeters;

public class C2024FeederIo implements FeederIo {
    private static final double BEAM_BREAK_THRESHOLD_VOLTAGE = 1.0;
    private static final double GEAR_RATIO = 1.0;
    private static final double ROLLER_DIAMETER = inchesToMeters(2.0);
    private final TalonFX motor;
    private final AnalogInput beamBreak;

    public C2024FeederIo(TalonFX motor, AnalogInput beamBreak) {
        this.motor = motor;
        this.beamBreak = beamBreak;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = (Math.PI * ROLLER_DIAMETER) / GEAR_RATIO;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(FeederInputs inputs) {
        inputs.angularVelocity = motor.getVelocity().getValueAsDouble();
        inputs.currentDrawAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.motorTemp = motor.getExpiration();
        inputs.beamBreakVoltage = beamBreak.getVoltage();
        inputs.beamBreakSignaled = beamBreak.getVoltage() > BEAM_BREAK_THRESHOLD_VOLTAGE;
    }

    @Override
    public void setVoltage(double voltage) {
        motor.set(voltage);
    }
}
