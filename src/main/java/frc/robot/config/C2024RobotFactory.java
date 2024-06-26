package frc.robot.config;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.subsystems.climber.C2024ClimberIo;
import frc.robot.subsystems.climber.ClimberIo;
import frc.robot.subsystems.drivetrain.CtreDrivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.feeder.C2024FeederIo;
import frc.robot.subsystems.feeder.FeederIo;
import frc.robot.subsystems.intake.C2024IntakeIo;
import frc.robot.subsystems.intake.IntakeIo;
import frc.robot.subsystems.led.LedCandleIo;
import frc.robot.subsystems.led.LedIo;
import frc.robot.subsystems.pivot.C2024PivotIo;
import frc.robot.subsystems.pivot.PivotIo;
import frc.robot.subsystems.shooter.C2024ShooterIo;
import frc.robot.subsystems.shooter.ShooterIo;
import frc.robot.subsystems.vision.C2024VisionIo;
import frc.robot.subsystems.vision.VisionIo;

public class C2024RobotFactory implements RobotFactory {
    @Override
    public ClimberIo createClimberIo() {
        return new C2024ClimberIo(
                new TalonFX(11, "3663"),
                new TalonFX(12, "3663"),
                new Servo(0),
                new Servo(1)
        );
    }

    @Override
    public DrivetrainIO createDrivetrainIO() {
        return new CtreDrivetrain(
                new CtreDrivetrain.Constants(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0)
                ),
                DrivetrainConstants.DRIVETRAIN_CONSTANTS,
                DrivetrainConstants.FRONT_LEFT,
                DrivetrainConstants.FRONT_RIGHT,
                DrivetrainConstants.BACK_LEFT,
                DrivetrainConstants.BACK_RIGHT
        );
    }

    @Override
    public IntakeIo createIntakeIo() {
        return new C2024IntakeIo(
                new TalonFX(9, "3663"),
                new TalonFX(13, "3663"),
                new TalonFX(14, "3663"),
                new DigitalInput(1)
        );
    }

    @Override
    public FeederIo createFeederIo() {
        return new C2024FeederIo(
                new TalonFX(3),
                new DigitalInput(0)
        );
    }

    @Override
    public LedIo createLedIo() {
        return new LedCandleIo(
                new CANdle(1)
        );
    }

    @Override
    public PivotIo createPivotIo() {
        return new C2024PivotIo(
                new TalonFX(10, "3663"),
                new TalonFX(4)
        );
    }

    @Override
    public ShooterIo createShooterIo() {
        return new C2024ShooterIo(
                new TalonFX(2),
                new TalonFX(1)
        );
    }

    @Override
    public VisionIo[] createVisionIo() {
        return new VisionIo[]{
                new C2024VisionIo(VisionConstants.LEFT_CAMERA_NAME, VisionConstants.LEFT_CAMERA_TRANSFORM),
                new C2024VisionIo(VisionConstants.RIGHT_CAMERA_NAME, VisionConstants.RIGHT_CAMERA_TRANSFORM)
        };
    }

    private static class DrivetrainConstants {
        // Both sets of gains need to be tuned to your individual robot.
        private static final double MODULE_WHEEL_INSET = Units.inchesToMeters(2.625);

        private static final double FRAME_X_LENGTH = Units.inchesToMeters(28.287024);
        private static final double INTAKE_X_OFFSET = Units.inchesToMeters(5.037024);
        private static final double FRONT_MODULE_X_OFFSET = FRAME_X_LENGTH / 2.0 - MODULE_WHEEL_INSET;
        private static final double BACK_MODULE_X_OFFSET = FRAME_X_LENGTH / 2.0 - INTAKE_X_OFFSET - MODULE_WHEEL_INSET;

        private static final double FRAME_Y_LENGTH = Units.inchesToMeters(27.5625);
        private static final double MODULE_Y_OFFSET = FRAME_Y_LENGTH / 2.0 - MODULE_WHEEL_INSET;

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs STEER_PID_CONSTANTS = new Slot0Configs()
                .withKP(100).withKI(0).withKD(0.2)
                .withKS(0).withKV(1.5).withKA(0);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs DRIVE_PID_CONSTANTS = new Slot0Configs()
                .withKP(0).withKI(0).withKD(0)
                .withKS(0).withKV(0).withKA(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final SwerveModule.ClosedLoopOutputType STEER_PID_OUTPUT_TYPE = SwerveModule.ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final SwerveModule.ClosedLoopOutputType DRIVE_PID_OUTPUT_TYPE = SwerveModule.ClosedLoopOutputType.Voltage;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double SLIP_CURRENT = 300.0;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double COUPLE_RATIO = 3.125;

        private static final double DRIVE_REDUCTION = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
        private static final double STEER_REDUCTION = 150.0 / 7.0;
        private static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        // ~5.76
        public static final double MAX_DRIVE_VELOCITY = DCMotor.getFalcon500Foc(1)
                .freeSpeedRadPerSec / DRIVE_REDUCTION * WHEEL_RADIUS;

        private static final boolean STEER_MOTOR_INVERTED = true;
        private static final boolean INVERT_LEFT_SIDE = false;
        private static final boolean INVERT_RIGHT_SIDE = false;

        private static final String CAN_BUS_NAME = "3663";
        private static final int PIGEON_ID = 0;


        // These are only used for simulation
        private static final double STEER_INERTIA = 0.00001;
        private static final double DRIVE_INERTIA = 0.001;
        // Simulated voltage necessary to overcome friction
        private static final double STEER_FRICTION_VOLTAGE = 0.25;
        private static final double DRIVE_FRICTION_VOLTAGE = 0.25;

        public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
                .withPigeon2Id(PIGEON_ID)
                .withCANbusName(CAN_BUS_NAME);

        private static final SwerveModuleConstantsFactory MODULE_CONSTANTS_FACTORY = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(DRIVE_REDUCTION)
                .withSteerMotorGearRatio(STEER_REDUCTION)
                .withWheelRadius(Units.metersToInches(WHEEL_RADIUS))
                .withSlipCurrent(SLIP_CURRENT)
                .withSteerMotorGains(STEER_PID_CONSTANTS)
                .withDriveMotorGains(DRIVE_PID_CONSTANTS)
                .withSteerMotorClosedLoopOutput(STEER_PID_OUTPUT_TYPE)
                .withDriveMotorClosedLoopOutput(DRIVE_PID_OUTPUT_TYPE)
                .withSpeedAt12VoltsMps(MAX_DRIVE_VELOCITY)
                .withSteerInertia(STEER_INERTIA)
                .withDriveInertia(DRIVE_INERTIA)
                .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
                .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE)
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
                .withCouplingGearRatio(COUPLE_RATIO)
                .withSteerMotorInverted(STEER_MOTOR_INVERTED);

        // Front Left
        private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
        private static final int FRONT_LEFT_STEER_MOTOR_ID = 2;
        private static final int FRONT_LEFT_ENCODER_ID = 1;
        private static final double FRONT_LEFT_ENCODER_OFFSET = Units.degreesToRotations(19.16015625);

        // Front Right
        private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
        private static final int FRONT_RIGHT_STEER_MOTOR_ID = 4;
        private static final int FRONT_RIGHT_ENCODER_ID = 2;
        private static final double FRONT_RIGHT_ENCODER_OFFSET = Units.degreesToRotations(-130.078125);

        // Back Left
        private static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
        private static final int BACK_LEFT_STEER_MOTOR_ID = 6;
        private static final int BACK_LEFT_ENCODER_ID = 3;
        private static final double BACK_LEFT_ENCODER_OFFSET = Units.degreesToRotations(111.26953125);

        // Back Right
        private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 7;
        private static final int BACK_RIGHT_STEER_MOTOR_ID = 8;
        private static final int BACK_RIGHT_ENCODER_ID = 4;
        private static final double BACK_RIGHT_ENCODER_OFFSET = Units.degreesToRotations(125.5078125);

        public static final SwerveModuleConstants FRONT_LEFT = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                FRONT_LEFT_STEER_MOTOR_ID, FRONT_LEFT_DRIVE_MOTOR_ID, FRONT_LEFT_ENCODER_ID, FRONT_LEFT_ENCODER_OFFSET, FRONT_MODULE_X_OFFSET, MODULE_Y_OFFSET, INVERT_LEFT_SIDE);
        public static final SwerveModuleConstants FRONT_RIGHT = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                FRONT_RIGHT_STEER_MOTOR_ID, FRONT_RIGHT_DRIVE_MOTOR_ID, FRONT_RIGHT_ENCODER_ID, FRONT_RIGHT_ENCODER_OFFSET, FRONT_MODULE_X_OFFSET, -MODULE_Y_OFFSET, INVERT_RIGHT_SIDE);
        public static final SwerveModuleConstants BACK_LEFT = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                BACK_LEFT_STEER_MOTOR_ID, BACK_LEFT_DRIVE_MOTOR_ID, BACK_LEFT_ENCODER_ID, BACK_LEFT_ENCODER_OFFSET, -BACK_MODULE_X_OFFSET, MODULE_Y_OFFSET, INVERT_LEFT_SIDE);
        public static final SwerveModuleConstants BACK_RIGHT = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                BACK_RIGHT_STEER_MOTOR_ID, BACK_RIGHT_DRIVE_MOTOR_ID, BACK_RIGHT_ENCODER_ID, BACK_RIGHT_ENCODER_OFFSET, -BACK_MODULE_X_OFFSET, -MODULE_Y_OFFSET, INVERT_RIGHT_SIDE);
    }

    private static class VisionConstants {
        // Photon cameras created with the camera-specific name
        public static final String LEFT_CAMERA_NAME = "left_camera";
        public static final String RIGHT_CAMERA_NAME = "right_camera";

        // Offsets of the cameras from the center of the robot
        private static final double LEFT_CAMERA_X = Units.inchesToMeters(5.25);
        private static final double LEFT_CAMERA_Y = Units.inchesToMeters(12.25);
        private static final double LEFT_CAMERA_Z = Units.inchesToMeters(16.5);
        private static final double LEFT_CAMERA_PITCH = Units.degreesToRadians(-15.00);
        private static final double LEFT_CAMERA_YAW = Units.degreesToRadians(15.220741);

        private static final double RIGHT_CAMERA_X = Units.inchesToMeters(5.25);
        private static final double RIGHT_CAMERA_Y = Units.inchesToMeters(-12.25);
        private static final double RIGHT_CAMERA_Z = Units.inchesToMeters(16.5);
        private static final double RIGHT_CAMERA_PITCH = Units.degreesToRadians(-10.00);
        private static final double RIGHT_CAMERA_YAW = Units.degreesToRadians(-15.220741);

        public static final Transform3d LEFT_CAMERA_TRANSFORM = new Transform3d(
                new Translation3d(LEFT_CAMERA_X, LEFT_CAMERA_Y, LEFT_CAMERA_Z),
                new Rotation3d(0.0, LEFT_CAMERA_PITCH, LEFT_CAMERA_YAW));
        public static final Transform3d RIGHT_CAMERA_TRANSFORM = new Transform3d(
                new Translation3d(RIGHT_CAMERA_X, RIGHT_CAMERA_Y, RIGHT_CAMERA_Z),
                new Rotation3d(0.0, RIGHT_CAMERA_PITCH, RIGHT_CAMERA_YAW));
    }
}
