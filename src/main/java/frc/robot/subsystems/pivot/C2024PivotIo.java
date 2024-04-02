package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class C2024PivotIo implements PivotIo {
    private static final double PIVOT_GEAR_RATIO = 54.0;
    private static final double AMP_GEAR_RATIO = 48.0 / 14.0;

    private static final Pivot.Constants CONSTANTS = new Pivot.Constants(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(125.0),
            Units.degreesToRadians(2.0),
            -1.0,
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(142.0),
            Units.degreesToRadians(2.0),
            -0.5,
            Units.degreesToRadians(70.0),
            Units.degreesToRadians(90.0)
    );

    private final TalonFX pivotMotor;
    private final TalonFX ampMotor;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    // Uses Slot 0 PID constants
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);
    private final StaticBrake brakeRequest = new StaticBrake();

    public C2024PivotIo(TalonFX pivotMotor, TalonFX ampMotor) {
        this.pivotMotor = pivotMotor;
        this.ampMotor = ampMotor;

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        pivotConfig.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO / (2.0 * Math.PI);
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = Units.degreesToRadians(1200);
        pivotConfig.MotionMagic.MotionMagicAcceleration = Units.degreesToRadians(720);

        // PID constants
        pivotConfig.Slot0.kP = 30.0;
        pivotConfig.Slot0.kI = 0.0;
        pivotConfig.Slot0.kD = 0.0;
        pivotConfig.Slot0.kS = 0.0;
        pivotConfig.Slot0.kV = 0.1;
        pivotConfig.Slot0.kA = 0.0;
        pivotConfig.Slot0.kG = 0.8;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // Applies config configuration onto the motor
        pivotMotor.getConfigurator().apply(pivotConfig);

        TalonFXConfiguration ampConfig = new TalonFXConfiguration();
        ampConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        ampConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        ampConfig.Feedback.SensorToMechanismRatio = AMP_GEAR_RATIO / (2.0 * Math.PI);
        ampConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        ampConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        ampConfig.MotionMagic.MotionMagicCruiseVelocity = Units.degreesToRadians(1500);
        ampConfig.MotionMagic.MotionMagicAcceleration = Units.degreesToRadians(900);

        // PID constants
        ampConfig.Slot0.kP = 20.0;
        ampConfig.Slot0.kI = 0.0;
        ampConfig.Slot0.kD = 0.0;
        ampConfig.Slot0.kS = 0.0;
        ampConfig.Slot0.kV = 1.0;
        ampConfig.Slot0.kA = 0.0;
        ampConfig.Slot0.kG = 0.0;
        ampConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        ampMotor.getConfigurator().apply(ampConfig);
    }

    @Override
    public Pivot.Constants getConstants() {
        return CONSTANTS;
    }

    @Override
    public void updateInputs(PivotInputs inputs) {
        inputs.pivotAngle = pivotMotor.getPosition().getValueAsDouble();
        inputs.pivotAngularVelocity = pivotMotor.getVelocity().getValueAsDouble();
        inputs.pivotAppliedVolts = pivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotCurrentDrawAmps = pivotMotor.getSupplyCurrent().getValueAsDouble();
        inputs.pivotMotorTemp = pivotMotor.getDeviceTemp().getValueAsDouble();

        inputs.ampAngle = ampMotor.getPosition().getValueAsDouble();
        inputs.ampAngularVelocity = ampMotor.getVelocity().getValueAsDouble();
        inputs.ampAppliedVolts = ampMotor.getMotorVoltage().getValueAsDouble();
        inputs.ampCurrentDrawAmps = ampMotor.getSupplyCurrent().getValueAsDouble();
        inputs.ampMotorTemp = ampMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setTargetAngle(double targetPivotAngle, double targetAmpAngle) {
        pivotMotor.setControl(positionRequest.withPosition(targetPivotAngle));
        ampMotor.setControl(positionRequest.withPosition(targetAmpAngle));
    }

    @Override
    public void resetPivotPosition(double pivotPosition) {
        pivotMotor.setPosition(pivotPosition);
    }

    @Override
    public void resetAmpPosition(double ampPosition) {
        ampMotor.setPosition(ampPosition);
    }

    @Override
    public void setVoltage(double pivotVoltage, double ampVoltage) {
        pivotMotor.setControl(voltageRequest.withOutput(pivotVoltage));
        ampMotor.setControl(voltageRequest.withOutput(ampVoltage));
    }

    @Override
    public void stop() {
        pivotMotor.setControl(brakeRequest);
        ampMotor.setControl(brakeRequest);
    }
}
