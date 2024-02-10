package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class C2024PivotIo implements PivotIo {
    private static final double GEAR_RATIO = 1.0;

    private final TalonFX primaryMotor;
    private final TalonFX secondaryMotor;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    // Uses Slot 0 PID constants
    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
    private final StaticBrake brakeRequest = new StaticBrake();

    public C2024PivotIo(TalonFX primaryMotor, TalonFX secondaryMotor) {
        this.primaryMotor = primaryMotor;
        this.secondaryMotor = secondaryMotor;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO * 2.0 * Math.PI;
        // PID constants
        config.Slot0.kP = 5.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        // Applies config configuration onto the motors
        primaryMotor.getConfigurator().apply(config);
        secondaryMotor.getConfigurator().apply(config);

        // Follower request for secondary motor
        Follower followRequest = new Follower(primaryMotor.getDeviceID(), false);
        secondaryMotor.setControl(followRequest);
    }

    @Override
    public void updateInputs(PivotInputs inputs) {
        inputs.inputVoltagePrimary = secondaryMotor.getSupplyVoltage().getValueAsDouble();
        inputs.inputVoltageSecondary = primaryMotor.getSupplyVoltage().getValueAsDouble();
        inputs.outputVoltagePrimary = secondaryMotor.getMotorVoltage().getValueAsDouble();
        inputs.outputVoltageSecondary = primaryMotor.getMotorVoltage().getValueAsDouble();

        // Motor Rotations * Gear Ratio * 2π = Pivot Radians
        inputs.currentAngleRad = primaryMotor.getPosition().getValueAsDouble();
        inputs.currentVelocityRadPerSec = primaryMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setTargetAngle(double targetRad) {
        primaryMotor.setControl(positionRequest.withPosition(targetRad));
    }

    @Override
    public void resetPosition(double position) {
        primaryMotor.setPosition(position);
    }

    @Override
    public void setVoltage(double voltage) {
        primaryMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void stop() {
        primaryMotor.setControl(brakeRequest);
    }
}
