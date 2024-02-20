package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.Robot;
import frc.robot.utility.SimUtil;

import static edu.wpi.first.math.util.Units.*;

public class SimClimberIo implements ClimberIo {
    private static final double UPDATE_PERIOD = Robot.defaultPeriodSecs;
    private static final double CURRENT_LIMIT = 40.0;

    //<editor-fold desc="State vector indices">
    private static final int STATE_LEFT_POSITION_INDEX = 0;
    private static final int STATE_LEFT_VELOCITY_INDEX = 1;
    private static final int STATE_RIGHT_POSITION_INDEX = 2;
    private static final int STATE_RIGHT_VELOCITY_INDEX = 3;
    //</editor-fold>

    //<editor-fold desc="Input vector indices">
    private static final int INPUT_LEFT_VOLTAGE_INDEX = 0;
    private static final int INPUT_RIGHT_VOLTAGE_INDEX = 1;
    //</editor-fold>

    //<editor-fold desc="Output vector indices">
    private static final int OUTPUT_LEFT_POSITION_INDEX = 0;
    private static final int OUTPUT_LEFT_VELOCITY_INDEX = 1;
    private static final int OUTPUT_RIGHT_POSITION_INDEX = 2;
    private static final int OUTPUT_RIGHT_VELOCITY_INDEX = 3;
    //</editor-fold>

    private final Config config;
    private final Climber.Constants constants;

    /**
     * Plant for the climber model
     */
    private final LinearSystem<N4, N2, N4> plant;
    private final LinearSystemSim<N4, N2, N4> sim;

    private final PIDController leftController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController rightController = new PIDController(10.0, 0.0, 0.0);

    //<editor-fold desc="State variables">
    private boolean leftLocked = true;
    private boolean rightLocked = true;

    private double leftTargetPosition = Double.NaN;
    private double rightTargetPosition = Double.NaN;

    private double leftTargetVoltage = Double.NaN;
    private double rightTargetVoltage = Double.NaN;
    //</editor-fold>

    public SimClimberIo() {
        this(new Config(
                DCMotor.getFalcon500(1),
                18.0,
                lbsToKilograms(60.0),
                inchesToMeters(1.75)
        ), new Climber.Constants(feetToMeters(4.0), -2.0));
    }

    public SimClimberIo(Config config, Climber.Constants constants) {
        this.config = config;
        this.constants = constants;
        this.plant = createPlantFromConfig(config);
        this.sim = new LinearSystemSim<>(plant);

        sim.setState(VecBuilder.fill(
                constants.maxArmHeight(),
                0.0,
                constants.maxArmHeight(),
                0.0
        ));
    }

    private static LinearSystem<N4, N2, N4> createPlantFromConfig(Config config) {
        Matrix<N4, N4> systemMatrix = new Matrix<>(Nat.N4(), Nat.N4());
        systemMatrix.set(STATE_LEFT_POSITION_INDEX, STATE_LEFT_VELOCITY_INDEX, 1.0);
        systemMatrix.set(STATE_LEFT_VELOCITY_INDEX, STATE_LEFT_VELOCITY_INDEX, -config.velocityCoefficient() / config.accelerationCoefficient());
        systemMatrix.set(STATE_RIGHT_POSITION_INDEX, STATE_RIGHT_VELOCITY_INDEX, 1.0);
        systemMatrix.set(STATE_RIGHT_VELOCITY_INDEX, STATE_RIGHT_VELOCITY_INDEX, -config.velocityCoefficient() / config.accelerationCoefficient());

        Matrix<N4, N2> inputMatrix = new Matrix<>(Nat.N4(), Nat.N2());
        inputMatrix.set(STATE_LEFT_VELOCITY_INDEX, INPUT_LEFT_VOLTAGE_INDEX, 1.0 / config.accelerationCoefficient());
        inputMatrix.set(STATE_RIGHT_VELOCITY_INDEX, INPUT_RIGHT_VOLTAGE_INDEX, 1.0 / config.accelerationCoefficient());

        Matrix<N4, N4> outputMatrix = Matrix.eye(Nat.N4());

        Matrix<N4, N2> feedthroughMatrix = new Matrix<>(Nat.N4(), Nat.N2());

        return new LinearSystem<>(systemMatrix, inputMatrix, outputMatrix, feedthroughMatrix);
    }

    @Override
    public Climber.Constants getConstants() {
        return constants;
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        double leftAppliedVolts = calculateAppliedVolts(
                sim.getOutput(OUTPUT_LEFT_POSITION_INDEX), sim.getOutput(OUTPUT_LEFT_VELOCITY_INDEX),
                leftLocked, leftTargetVoltage, leftTargetPosition, leftController);
        double rightAppliedVolts = calculateAppliedVolts(
                sim.getOutput(OUTPUT_RIGHT_POSITION_INDEX), sim.getOutput(OUTPUT_RIGHT_VELOCITY_INDEX),
                rightLocked, rightTargetVoltage, rightTargetPosition, rightController);

        sim.setInput(leftAppliedVolts, rightAppliedVolts);
        sim.update(UPDATE_PERIOD);

        // Constrain the movement of the climber
        {
            boolean clamped = false;
            var output = sim.getOutput();
            double leftPosition = output.get(OUTPUT_LEFT_POSITION_INDEX, 0);
            double rightPosition = output.get(OUTPUT_RIGHT_POSITION_INDEX, 0);
            if (leftPosition < 0 || rightPosition > constants.maxArmHeight()) {
                output.set(OUTPUT_LEFT_POSITION_INDEX, 0, MathUtil.clamp(leftPosition, 0.0, constants.maxArmHeight()));
                output.set(OUTPUT_LEFT_VELOCITY_INDEX, 0, 0.0);
                clamped = true;
            }
            if (rightPosition < 0 || rightPosition > constants.maxArmHeight()) {
                output.set(OUTPUT_RIGHT_POSITION_INDEX, 0, MathUtil.clamp(rightPosition, 0.0, constants.maxArmHeight()));
                output.set(OUTPUT_RIGHT_VELOCITY_INDEX, 0, 0.0);
                clamped = true;
            }

            if (clamped) {
                sim.setState(output.copy());
            }
        }

        inputs.leftPosition = sim.getOutput(OUTPUT_LEFT_POSITION_INDEX);
        inputs.leftVelocity = sim.getOutput(OUTPUT_LEFT_VELOCITY_INDEX);
        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.leftCurrentDrawAmps = Math.abs(config.motor().getCurrent(inputs.leftVelocity * config.pulleyRadius() / config.reduction(), inputs.leftAppliedVolts));
        inputs.leftLocked = leftLocked;

        inputs.rightPosition = sim.getOutput(OUTPUT_RIGHT_POSITION_INDEX);
        inputs.rightVelocity = sim.getOutput(OUTPUT_RIGHT_VELOCITY_INDEX);
        inputs.rightAppliedVolts = rightAppliedVolts;
        inputs.rightCurrentDrawAmps = Math.abs(config.motor().getCurrent(inputs.rightVelocity * config.pulleyRadius() / config.reduction(), inputs.rightAppliedVolts));
        inputs.rightLocked = rightLocked;
    }

    private double calculateAppliedVolts(
            double currentPosition,
            double currentVelocity,
            boolean locked, double targetVoltage, double targetPosition,
            PIDController controller) {
        if (locked) {
            return 0.0;
        }

        double voltage = 0.0;
        if (Double.isFinite(targetVoltage)) {
            voltage = targetVoltage;
        }
        if (Double.isFinite(targetPosition)) {
            voltage = controller.calculate(currentPosition, targetPosition);
        }
        return SimUtil.applyCurrentLimit(config.motor(),
                currentVelocity / config.pulleyRadius() / config.reduction(),
                voltage,
                CURRENT_LIMIT);
    }

    @Override
    public void setVoltage(double voltageLeft, double voltageRight) {
        leftTargetPosition = Double.NaN;
        rightTargetPosition = Double.NaN;
        leftTargetVoltage = voltageLeft;
        rightTargetVoltage = voltageRight;
    }

    @Override
    public void setTargetPosition(double leftHeight, double rightHeight) {
        leftTargetPosition = leftHeight;
        rightTargetPosition = rightHeight;
        leftTargetVoltage = Double.NaN;
        rightTargetVoltage = Double.NaN;
    }

    @Override
    public void setLocked(boolean lockedLeft, boolean lockedRight) {
        leftLocked = lockedLeft;
        rightLocked = lockedRight;
    }

    public record Config(
            DCMotor motor,
            double reduction,
            double mass,
            double pulleyRadius
    ) {
        public double velocityCoefficient() {
            return reduction / (pulleyRadius * motor.KvRadPerSecPerVolt);
        }

        public double accelerationCoefficient() {
            return motor.rOhms * pulleyRadius * mass / (reduction * motor.KtNMPerAmp);
        }
    }
}
