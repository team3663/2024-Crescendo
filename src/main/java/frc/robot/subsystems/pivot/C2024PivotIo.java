package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class C2024PivotIo implements PivotIo {
    private static final double GEAR_RATIO = 1.0;

    private final TalonFX primaryMotor;
    private final TalonFX secondaryMotor;

    // Uses Slot 0 PID constants
    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

    public C2024PivotIo(TalonFX primaryMotor, TalonFX secondaryMotor) {
        this.primaryMotor = primaryMotor;
        this.secondaryMotor = secondaryMotor;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO * 2.0 * Math.PI;
        // PID constants
        config.Slot0.kP = 5.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        primaryMotor.getConfigurator().apply(config);
        secondaryMotor.getConfigurator().apply(config);

        // Follower request for secondary motor
        Follower followRequest = new Follower(primaryMotor.getDeviceID(), false);
        secondaryMotor.setControl(followRequest);
    }

    @Override
    public void updateInputs(PivotInputs inputs) {
        inputs.inputVoltageRight = secondaryMotor.getSupplyVoltage().getValueAsDouble();
        inputs.inputVoltageLeft = primaryMotor.getSupplyVoltage().getValueAsDouble();
        inputs.outputVoltageRight = secondaryMotor.getMotorVoltage().getValueAsDouble();
        inputs.outputVoltageLeft = primaryMotor.getMotorVoltage().getValueAsDouble();

        // Motor Rotations * Gear Ratio * 2π = Pivot Radians
        inputs.currentAngleRad = primaryMotor.getPosition().getValueAsDouble();
        inputs.currentVelocityRadPerSec = primaryMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setTargetAngle(double targetRad) {
        primaryMotor.setControl(positionRequest.withPosition(targetRad));
    }
}
