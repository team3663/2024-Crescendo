package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class C2024PivotIo implements PivotIo {
    private static final double GEAR_RATIO = 54.0;

    private static final Pivot.Constants CONSTANTS = new Pivot.Constants(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(125.0),
            Units.degreesToRadians(2.0),
            -1.0
    );

    private final TalonFX motor;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    // Uses Slot 0 PID constants
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);
    private final StaticBrake brakeRequest = new StaticBrake();

    public C2024PivotIo(TalonFX motor) {
        this.motor = motor;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO / (2.0 * Math.PI);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.MotionMagic.MotionMagicCruiseVelocity = Units.degreesToRadians(1200);
        config.MotionMagic.MotionMagicAcceleration = Units.degreesToRadians(5000);

        // PID constants
        config.Slot0.kP = 30.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.1;
        config.Slot0.kA = 0.0;
        config.Slot0.kG = 0.8;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // Applies config configuration onto the motor
        motor.getConfigurator().apply(config);
    }

    @Override
    public Pivot.Constants getConstants() {
        return CONSTANTS;
    }

    @Override
    public void updateInputs(PivotInputs inputs) {
        inputs.angle = motor.getPosition().getValueAsDouble();
        inputs.angularVelocity = motor.getVelocity().getValueAsDouble();
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.currentDrawAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.motorTemp = motor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setTargetAngle(double targetRad) {
        motor.setControl(positionRequest.withPosition(targetRad));
    }

    @Override
    public void resetPosition(double position) {
        motor.setPosition(position);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void stop() {
        motor.setControl(brakeRequest);
    }
}
