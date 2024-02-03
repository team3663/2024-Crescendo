package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class c2024PivotIO implements PivotIO{

    private double targetRad = 0;
    private final TalonFX motor;
    private final TalonFXConfiguration motorController = new TalonFXConfiguration();
    private static final double gearRatio = 1.0;

    public c2024PivotIO(TalonFX motor){
        this.motor = motor;
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.inputVoltage = motor.getSupplyVoltage().getValueAsDouble();
        inputs.outputVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.targetAngleRad = targetRad;

        // Rotations * Gear Ratio * 2π = Pivot Radians
        inputs.currentAngleRad = motor.getPosition().getValueAsDouble() * gearRatio * 2 * Math.PI;
    }

    @Override
    public void setTargetAngle(double targetRad) {
        this.targetRad = targetRad;
        motor.setPosition(targetRad/(gearRatio * Math.PI));
    }
}
