package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class c2024PivotIo implements PivotIo {

    private double targetRad = 0;
    private final TalonFX l_motor;
    private final TalonFX r_motor;
    private static final double gearRatio = 1.0;

    public c2024PivotIo(TalonFX l_motor, TalonFX r_motor){
        this.l_motor = l_motor;
        this.r_motor = r_motor;

        // Uses SLot 0 PID constants
        PositionVoltage request =
                new PositionVoltage(radToRot(targetRad)).withSlot(0);

        // PID constants
        Slot0Configs pid = new Slot0Configs();
        pid.kP = 5;
        pid.kI = 0;
        pid.kD = 0;

        // Follower request for right motor
        Follower followRequest = new Follower(l_motor.getDeviceID(), true);

        l_motor.setControl(request);
        r_motor.setControl(followRequest);
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.inputVoltageRight = r_motor.getSupplyVoltage().getValueAsDouble();
        inputs.inputVoltageLeft = l_motor.getSupplyVoltage().getValueAsDouble();
        inputs.outputVoltageRight = r_motor.getMotorVoltage().getValueAsDouble();
        inputs.outputVoltageLeft = l_motor.getMotorVoltage().getValueAsDouble();

        // Motor Rotations * Gear Ratio * 2π = Pivot Radians
        inputs.currentAngleRadRight = -r_motor.getPosition().getValueAsDouble() * gearRatio * 2 * Math.PI;
        inputs.currentAngleRadLeft = l_motor.getPosition().getValueAsDouble() * gearRatio * 2 * Math.PI;

        inputs.targetAngleRad = targetRad;
    }

    @Override
    public void setTargetAngle(double targetRad) {
        this.targetRad = targetRad;
        l_motor.setPosition(radToRot(targetRad));
    }

    // Motor Rotations = Pivot Radians / (Gear Ratio * 2π)
    public double radToRot(double rad) {
        return rad / (gearRatio * 2 * Math.PI);
    }
}
