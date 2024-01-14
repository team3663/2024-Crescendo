package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.math.util.Units.feetToMeters;

public class Mk4SwerveModule implements SwerveModuleIO {
    private static final double VELOCITY_CONSTANT = 12.0 / feetToMeters(16.5);

    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANcoder steerEncoder;

    public Mk4SwerveModule(TalonFX driveMotor, TalonFX steerMotor, CANcoder steerEncoder, double steerAngleOffset) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.steerEncoder = steerEncoder;

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = Units.radiansToRotations(steerAngleOffset);

        steerEncoder.getConfigurator().apply(encoderConfig);

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

        steerConfig.CurrentLimits.StatorCurrentLimit = 10.0;
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        steerConfig.Feedback.FeedbackRemoteSensorID = steerEncoder.getDeviceID();

        steerConfig.Feedback.SensorToMechanismRatio = 1.0; // 1:1
        steerConfig.Feedback.RotorToSensorRatio = 12.8;

        steerConfig.Slot0.kP = 50;
        steerConfig.Slot0.kD = 1;

        steerMotor.getConfigurator().apply(steerConfig);
    }

    @Override
    public void updateInputs(Inputs inputs) {
        StatusSignal<Double> steerAngleSignal = steerMotor.getPosition();

        steerAngleSignal.refresh();

        StatusSignal<Double> encoderAngleSignal = steerEncoder.getPosition();

        encoderAngleSignal.refresh();

        inputs.steerAngleRad = Units.rotationsToRadians(steerAngleSignal.getValue());
        inputs.encoderAngleRad = Units.rotationsToRadians(encoderAngleSignal.getValue());
    }

    @Override
    public void setTargetState(SwerveModuleState state) {
        steerMotor.setControl(new PositionVoltage(state.angle.getRotations()));
        driveMotor.setControl(new VoltageOut(state.speedMetersPerSecond * VELOCITY_CONSTANT));
    }
}
