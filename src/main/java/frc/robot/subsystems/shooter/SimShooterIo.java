package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;
import frc.robot.utility.SimUtil;

public class SimShooterIo implements ShooterIo {
    private static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
    private static final double GEAR_RATIO = 1.0;

    private static final double MOMENT_OF_INERTIA = 0.05;

    private static final double CURRENT_LIMIT = 40.0;

    private final LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(MOTOR, MOMENT_OF_INERTIA, GEAR_RATIO);
    private final LinearPlantInversionFeedforward<N1, N1, N1> feedforward = new LinearPlantInversionFeedforward<>(plant, Robot.defaultPeriodSecs);

    private final PIDController controller = new PIDController(1.0, 0.0, 0.0);
    private final FlywheelSim sim = new FlywheelSim(plant, MOTOR, GEAR_RATIO, VecBuilder.fill(0.01));

    private State state = State.STOPPED;

    private double nextTargetVelocity = 0.0;

    @Override
    public void updateInputs(ShooterInputs inputs) {
        double voltage = MathUtil.clamp(switch (state) {
            case STOPPED -> 0;
            case VELOCITY -> {
                double lastTargetVelocity = controller.getSetpoint();

                double controllerEffort = controller.calculate(sim.getAngularVelocityRadPerSec());
                controller.setSetpoint(nextTargetVelocity);
                double feedforwardEffort = feedforward.calculate(VecBuilder.fill(lastTargetVelocity), VecBuilder.fill(nextTargetVelocity))
                        .get(0, 0);

                double resultVoltage = controllerEffort + feedforwardEffort;

                // Current limiting
                // If the estimated current draw is going to be larger than the limit, calculate what voltage
                // would be exactly at our limit and use that.
                yield SimUtil.applyCurrentLimit(MOTOR, sim.getAngularVelocityRadPerSec()*GEAR_RATIO, resultVoltage, CURRENT_LIMIT);
            }
        }, -12.0, 12.0);

        sim.setInputVoltage(voltage);
        sim.update(Robot.defaultPeriodSecs);

        inputs.upperAngularVelocity = sim.getAngularVelocityRadPerSec();
        inputs.upperAppliedVolts = voltage;
        inputs.upperCurrentDrawAmps = sim.getCurrentDrawAmps();

        inputs.lowerAngularVelocity = sim.getAngularVelocityRadPerSec();
        inputs.lowerAppliedVolts = voltage;
        inputs.lowerCurrentDrawAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setTargetVelocity(double targetVelocity) {
        if (state != State.VELOCITY) {
            controller.reset();
        }

        nextTargetVelocity = targetVelocity;
        state = State.VELOCITY;
    }

    @Override
    public void stop() {
        state = State.STOPPED;
    }

    private enum State {
        STOPPED,
        VELOCITY
    }
}
