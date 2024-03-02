package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;

public class C2024ClimberIo implements ClimberIo {

    private static final double GEAR_RATIO = 9.66;
    private static final double PULLEY_RADIUS = Units.inchesToMeters(0.6);
    private static final double LOCKED_THRESHOLD = 0.5;

    private final TalonFX rightMotor;
    private final TalonFX leftMotor;
    private final Servo leftServo;
    private final Servo rightServo;

    private final StaticBrake brakeRequest = new StaticBrake();
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public C2024ClimberIo(TalonFX leftMotor, TalonFX rightMotor, Servo leftServo, Servo rightServo) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.leftServo = leftServo;
        this.rightServo = rightServo;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO / (2.0 * Math.PI * PULLEY_RADIUS);
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);

        leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        rightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    }

    @Override
    public Climber.Constants getConstants() {
        return new Climber.Constants(0.75, -1.0);
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.leftPosition = leftMotor.getPosition().getValueAsDouble();
        inputs.leftVelocity = leftMotor.getVelocity().getValueAsDouble();
        inputs.leftAppliedVolts = leftMotor.getMotorVoltage().getValueAsDouble();
        inputs.leftCurrentDrawAmps = leftMotor.getSupplyCurrent().getValueAsDouble();
        inputs.leftMotorTemp = leftMotor.getDeviceTemp().getValueAsDouble();
        inputs.leftLocked = leftServo.getPosition() >= LOCKED_THRESHOLD;

        inputs.rightPosition = rightMotor.getPosition().getValueAsDouble();
        inputs.rightVelocity = rightMotor.getVelocity().getValueAsDouble();
        inputs.rightAppliedVolts = rightMotor.getMotorVoltage().getValueAsDouble();
        inputs.rightCurrentDrawAmps = rightMotor.getSupplyCurrent().getValueAsDouble();
        inputs.rightMotorTemp = rightMotor.getDeviceTemp().getValueAsDouble();
        inputs.rightLocked = rightServo.getPosition() >= LOCKED_THRESHOLD;
    }

    @Override
    public void stop() {
        leftMotor.setControl(brakeRequest);
        rightMotor.setControl(brakeRequest);
    }

    @Override
    public void resetPosition() {
        leftMotor.setPosition(0.0);
        rightMotor.setPosition(0.0);
    }

    @Override
    public void setTargetPosition(double leftHeight, double rightHeight) {
//        leftMotor.setControl(positionRequest.withPosition(leftHeight));
//        rightMotor.setControl(positionRequest.withPosition(rightHeight));
    }

    public void setVoltage(double leftVoltage, double rightVoltage) {
        leftMotor.setControl(voltageRequest.withOutput(leftVoltage));
        rightMotor.setControl(voltageRequest.withOutput(rightVoltage));
    }

    @Override
    public void setLocked(boolean leftLocked, boolean rightLocked) {
        leftServo.setPosition(leftLocked ? 1.0 : 0.0);
        rightServo.setPosition(rightLocked ? 1.0 : 0.0);
    }
}
