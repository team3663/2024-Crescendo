package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.AngleUtil;
import frc.robot.utility.FireControlSystem;
import frc.robot.utility.FiringSolution;
import frc.robot.utility.SubwooferRotations;
import org.littletonrobotics.junction.Logger;

import java.util.Collections;
import java.util.Comparator;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond;
import static frc.robot.Constants.PIVOT_POST_SHOOT_MOVEMENT_DELAY;

public class CommandFactory {
    private final Climber climber;
    private final Drivetrain drivetrain;
    private final Feeder feeder;
    private final Intake intake;
    private final Led led;
    private final Pivot pivot;
    private final Shooter shooter;
    private final Vision vision;

    private final FireControlSystem fireControlSystem = new FireControlSystem();

    public CommandFactory(
            Climber climber,
            Drivetrain drivetrain,
            Feeder feeder,
            Intake intake,
            Led led,
            Pivot pivot,
            Shooter shooter,
            Vision vision
    ) {
        this.climber = climber;
        this.drivetrain = drivetrain;
        this.feeder = feeder;
        this.intake = intake;
        this.led = led;
        this.pivot = pivot;
        this.shooter = shooter;
        this.vision = vision;
    }

    /**
     * Intakes a note from the ground and loads it into the feeder.
     * <p>
     * Uses:
     * <ul>
     *     <li>Feeder</li>
     *     <li>Intake</li>
     * </ul>
     * Finish condition: When a note has been loaded into the feeder.
     */
    public Command intakeAndLoad() {
        // Spin both the feeder and the intake until we detect a piece in the feeder
        return Commands.parallel(
                Commands.waitSeconds(0.25).andThen(Commands.waitUntil(feeder::isDetected))
                        .deadlineWith(intake.withVoltage(4.0)),
                feeder.runWithVoltage(3.0)
                        .until(feeder::isDetected)
        );
    }

    public Command shoot() {
        return Commands.deadline(
                Commands.sequence(
                        Commands.waitUntil(shooter::atTargetVelocity),
                        feeder.runWithVoltage(8.0)
                ).until(feeder::isNotDetected),
                shooter.withVelocity(Units.rotationsPerMinuteToRadiansPerSecond(2500))
        );
    }

    /**
     * Aims using a firing solution and fires when able to.
     * <p>
     * Uses:
     * <ul>
     *     <li>Drivetrain</li>
     *     <li>Intake</li>
     *     <li>Pivot</li>
     *     <li>Shooter</li>
     * </ul>
     *
     * @param allowedToFireSupplier   Whether the command is allowed to fire. When true, the shooter will fire when
     *                                the calculated firing solution is achieved.
     * @param fireCommand             The command to run when firing.
     * @param xVelocitySupplier       A supplier for the drivetrain's X velocity.
     * @param yVelocitySupplier       A supplier for the drivetrain's Y velocity.
     * @param angularVelocitySupplier A supplier for the drivetrain's angular velocity. Used when no firing solution
     *                                can be calculated.
     */
    public Command aimAndFire(
            Supplier<Optional<FiringSolution>> firingSolutionSupplier,
            Command fireCommand,
            BooleanSupplier allowedToFireSupplier,
            DoubleSupplier xVelocitySupplier,
            DoubleSupplier yVelocitySupplier,
            DoubleSupplier angularVelocitySupplier) {
        final double ALLOWABLE_ROBOT_ROTATION_ERROR = degreesToRadians(3.0);
        final double ALLOWABLE_PIVOT_ANGLE_ERROR = degreesToRadians(2.5);
        final double ALLOWABLE_SHOOTER_VELOCITY_ERROR = rotationsPerMinuteToRadiansPerSecond(75.0);

        // The current firing solution to use
        // Because we want to re-calculate this inside a lambda expression, this is an array of length 1
        //noinspection unchecked
        Optional<FiringSolution>[] firingSolution = new Optional[]{Optional.empty()};

        // Note: We're building the command groups manually, rather than the functional method because most of our
        // commands run in one of two groups:
        // A command will run in the pre-fire group if any of it's required subsystems are also required by the fire command
        // Or a command will run in the main group if none of it's required subsystems are also required by the fire command
        // The main group runs the entire duration of the aimAndFire command, while the pre-fire group only runs before we
        // run the fire sequence
        ParallelCommandGroup mainGroup = new ParallelCommandGroup();
        ParallelCommandGroup preFireGroup = new ParallelCommandGroup();

        Command driveCommand = drivetrain.driveWithOptionalAngle(xVelocitySupplier, yVelocitySupplier, angularVelocitySupplier, () -> firingSolution[0].flatMap(FiringSolution::robotRotation));
        if (Collections.disjoint(driveCommand.getRequirements(), fireCommand.getRequirements()))
            mainGroup.addCommands(driveCommand);
        else
            preFireGroup.addCommands(driveCommand);

        Command pivotCommand = pivot.follow(() -> firingSolution[0].map(FiringSolution::pivotAngle).orElse(pivot.getConstants().minAngle()));
        if (Collections.disjoint(pivotCommand.getRequirements(), fireCommand.getRequirements()))
            mainGroup.addCommands(pivotCommand);
        else
            preFireGroup.addCommands(pivotCommand);

        Command shooterCommand = shooter.withVelocity(() -> firingSolution[0].map(FiringSolution::shooterVelocity).orElse(Double.NaN));
        if (Collections.disjoint(shooterCommand.getRequirements(), fireCommand.getRequirements()))
            mainGroup.addCommands(shooterCommand);
        else
            preFireGroup.addCommands(shooterCommand);

        // Assemble the main command
        return Commands.deadline(
                        // Only fire when all are met: we have a valid targeting solution, we are facing the correct direction,
                        // the pivot is at the correct angle, the shooter is at the correct velocity, and we are allowed to fire
                        Commands.waitUntil(() -> firingSolution[0].isPresent() &&
                                (firingSolution[0].get().robotRotation().isEmpty() || AngleUtil.angleDifference(drivetrain.getPose().getRotation(), firingSolution[0].get().robotRotation().get()) < ALLOWABLE_ROBOT_ROTATION_ERROR) &&
                                MathUtil.isNear(firingSolution[0].get().pivotAngle(), pivot.getAngle(), ALLOWABLE_PIVOT_ANGLE_ERROR) &&
                                MathUtil.isNear(firingSolution[0].get().shooterVelocity(), shooter.getVelocity(), ALLOWABLE_SHOOTER_VELOCITY_ERROR) &&
                                allowedToFireSupplier.getAsBoolean()
                        ).deadlineWith(preFireGroup).andThen(fireCommand),
                        // Constantly re-calculate the firing solution
                        Commands.run(() -> firingSolution[0] = firingSolutionSupplier.get()),

                        // Record status
                        Commands.run(() -> {
                            Logger.recordOutput("FCS/HasSolution", firingSolution[0].isPresent());
                            Logger.recordOutput("FCS/TargetRobotRotation", firingSolution[0].flatMap(FiringSolution::robotRotation).orElse(null));
                            Logger.recordOutput("FCS/TargetPivotAngle", firingSolution[0].map(FiringSolution::pivotAngle).orElse(Double.NaN));
                            Logger.recordOutput("FCS/TargetShooterVelocity", firingSolution[0].map(FiringSolution::shooterVelocity).orElse(Double.NaN));

                            Logger.recordOutput("FCS/AtTargetRobotRotation", firingSolution[0].flatMap(FiringSolution::robotRotation)
                                    .map(rotation -> AngleUtil.angleDifference(drivetrain.getPose().getRotation(), rotation) < ALLOWABLE_ROBOT_ROTATION_ERROR).orElse(false));
                            Logger.recordOutput("FCS/AtTargetPivotAngle", firingSolution[0].map(FiringSolution::pivotAngle)
                                    .map(angle -> MathUtil.isNear(angle, pivot.getAngle(), ALLOWABLE_PIVOT_ANGLE_ERROR)).orElse(false));
                            Logger.recordOutput("FCS/AtTargetShooterVelocity", firingSolution[0].map(FiringSolution::shooterVelocity)
                                    .map(velocity -> MathUtil.isNear(velocity, shooter.getVelocity(), ALLOWABLE_SHOOTER_VELOCITY_ERROR)).orElse(false));
                            Logger.recordOutput("FCS/AllowedToFire", allowedToFireSupplier.getAsBoolean());
                        }),
                        mainGroup)
                // Clear the old firing solution before starting
                .beforeStarting(() -> firingSolution[0] = Optional.empty());
    }

    /**
     * Aims at the current alliance's amp and fires when able to.
     * <p>
     * Uses:
     * <ul>
     *     <li>Drivetrain</li>
     *     <li>Feeder</li>
     *     <li>Intake</li>
     *     <li>Pivot</li>
     *     <li>Shooter</li>
     * </ul>
     *
     * @param allowedToFireSupplier   Whether the command is allowed to fire. When true, the shooter will fire when
     *                                the calculated firing solution is achieved.
     * @param xVelocitySupplier       A supplier for the drivetrain's X velocity.
     * @param yVelocitySupplier       A supplier for the drivetrain's Y velocity.
     * @param angularVelocitySupplier A supplier for the drivetrain's angular velocity. Used when no firing solution
     *                                can be calculated.
     */
    public Command aimAndFireAtAmp(
            BooleanSupplier allowedToFireSupplier,
            DoubleSupplier xVelocitySupplier,
            DoubleSupplier yVelocitySupplier,
            DoubleSupplier angularVelocitySupplier) {
        final double PIVOT_ANGLE = Units.degreesToRadians(119.0);
        final double SHOOTER_VELOCITY = Units.rotationsPerMinuteToRadiansPerSecond(1500.0);

        return aimAndFire(() -> DriverStation.getAlliance().map(Constants::getAmpRotationForAlliance)
                        .map(robotRotation -> new FiringSolution(robotRotation, PIVOT_ANGLE, SHOOTER_VELOCITY)),
                // Move slightly away from the wall and then fire
                drivetrain.driveRobotOriented(() -> 0.1, () -> 0.0, () -> 0.0).withTimeout(0.25).andThen(feeder.runWithVoltage(12.0)).until(feeder::isNotDetected),
                allowedToFireSupplier,
                xVelocitySupplier,
                yVelocitySupplier,
                angularVelocitySupplier
        );
    }

    /**
     * Aims at the current alliance's speaker and fires when able to.
     * <p>
     * Uses:
     * <ul>
     *     <li>Drivetrain</li>
     *     <li>Feeder</li>
     *     <li>Intake</li>
     *     <li>Pivot</li>
     *     <li>Shooter</li>
     * </ul>
     *
     * @param allowedToFireSupplier   Whether the command is allowed to fire. When true, the shooter will fire when
     *                                the calculated firing solution is achieved.
     * @param xVelocitySupplier       A supplier for the drivetrain's X velocity.
     * @param yVelocitySupplier       A supplier for the drivetrain's Y velocity.
     * @param angularVelocitySupplier A supplier for the drivetrain's angular velocity. Used when no firing solution
     *                                can be calculated.
     */
    public Command aimAndFireAtSpeaker(BooleanSupplier allowedToFireSupplier,
                                       DoubleSupplier xVelocitySupplier,
                                       DoubleSupplier yVelocitySupplier,
                                       DoubleSupplier angularVelocitySupplier) {
        return aimAndFire(
                // Firing solution calculator
                () -> DriverStation.getAlliance().map(Constants::getSpeakerPositionForAlliance)
                        .map(speakerPosition -> {
                            Pose2d robotPose = drivetrain.getPose();

                            Logger.recordOutput("FCS/TargetDifference", speakerPosition.minus(robotPose.getTranslation()));
                            Logger.recordOutput("FCS/TargetDistance", speakerPosition.getDistance(robotPose.getTranslation()));


                            return fireControlSystem.calculate(drivetrain.getPose(), speakerPosition);
                        }),
                // Fire command
                Commands.deadline(
                        Commands.waitUntil(feeder::isNotDetected).andThen(Commands.waitSeconds(PIVOT_POST_SHOOT_MOVEMENT_DELAY)),
                        feeder.runWithVoltage(12.0)),
                allowedToFireSupplier,
                xVelocitySupplier,
                yVelocitySupplier,
                angularVelocitySupplier
        );
    }

    /**
     * Aims at the current alliance's amp and fires when able to.
     * <p>
     * Uses:
     * <ul>
     *     <li>Drivetrain</li>
     *     <li>Feeder</li>
     *     <li>Intake</li>
     *     <li>Pivot</li>
     *     <li>Shooter</li>
     * </ul>
     *
     * @param allowedToFireSupplier   Whether the command is allowed to fire. When true, the shooter will fire when
     *                                the calculated firing solution is achieved.
     * @param xVelocitySupplier       A supplier for the drivetrain's X velocity.
     * @param yVelocitySupplier       A supplier for the drivetrain's Y velocity.
     * @param angularVelocitySupplier A supplier for the drivetrain's angular velocity. Used when no firing solution
     *                                can be calculated.
     */
    public Command aimAndFireAtSubwoofer(
            BooleanSupplier allowedToFireSupplier,
            DoubleSupplier xVelocitySupplier,
            DoubleSupplier yVelocitySupplier,
            DoubleSupplier angularVelocitySupplier) {
        // Constants to use when at the front of the subwoofer
        final double FRONT_PIVOT_ANGLE = Units.degreesToRadians(50.0);
        final double FRONT_SHOOTER_VELOCITY = Units.rotationsPerMinuteToRadiansPerSecond(2500.0);
        // Constants to use when at the side of the subwoofer
        final double SIDE_PIVOT_ANGLE = Units.degreesToRadians(50.0);
        final double SIDE_SHOOTER_VELOCITY = Units.rotationsPerMinuteToRadiansPerSecond(3000.0);

        // Constants to use when at the front of the subwoofer when robot is backwards
        final double FRONT_REVERSE_PIVOT_ANGLE = Units.degreesToRadians(110.0);
        final double FRONT_REVERSE_SHOOTER_VELOCITY = Units.rotationsPerMinuteToRadiansPerSecond(2500.0);
        // Constants to use when at the side of the subwoofer when robot is backwards
        final double SIDE_REVERSE_PIVOT_ANGLE = Units.degreesToRadians(115.0);
        final double SIDE_REVERSE_SHOOTER_VELOCITY = Units.rotationsPerMinuteToRadiansPerSecond(3000.0);

        return aimAndFire(() -> DriverStation.getAlliance().map(Constants::getSubwooferRotationsForAlliance).map(subwooferRotations -> {
                    Rotation2d robotRotation = drivetrain.getPose().getRotation();

                    SubwooferRotations reversedSubwooferRotations = new SubwooferRotations(
                            subwooferRotations.front().rotateBy(Rotation2d.fromDegrees(180.0)),
                            subwooferRotations.ampSide().rotateBy(Rotation2d.fromDegrees(180.0)),
                            subwooferRotations.sourceSide().rotateBy(Rotation2d.fromDegrees(180.0)));

                    Rotation2d nearestRotation =
                            Stream.of(subwooferRotations.front(), subwooferRotations.ampSide(), subwooferRotations.sourceSide(),
                                            reversedSubwooferRotations.front(), reversedSubwooferRotations.ampSide(), reversedSubwooferRotations.sourceSide())
                                    .min(Comparator.comparingDouble(rotation -> AngleUtil.angleDifference(robotRotation, rotation)))
                                    .orElse(subwooferRotations.front());

                    Logger.recordOutput("FCS/NearestSubwooferRotation", nearestRotation);
                    Logger.recordOutput("FCS/NearestSubwooferRotationDifference", AngleUtil.angleDifference(robotRotation, nearestRotation));

                    if (nearestRotation.equals(subwooferRotations.ampSide()) || nearestRotation.equals(subwooferRotations.sourceSide()))
                        return new FiringSolution(Optional.empty(), SIDE_PIVOT_ANGLE, SIDE_SHOOTER_VELOCITY);
                    else if (nearestRotation.equals(reversedSubwooferRotations.front()))
                        return new FiringSolution(Optional.empty(), FRONT_REVERSE_PIVOT_ANGLE, FRONT_REVERSE_SHOOTER_VELOCITY);
                    else if (nearestRotation.equals(reversedSubwooferRotations.ampSide()) || nearestRotation.equals(reversedSubwooferRotations.sourceSide()))
                        return new FiringSolution(Optional.empty(), SIDE_REVERSE_PIVOT_ANGLE, SIDE_REVERSE_SHOOTER_VELOCITY);
                    else
                        return new FiringSolution(Optional.empty(), FRONT_PIVOT_ANGLE, FRONT_SHOOTER_VELOCITY);
                }),
                Commands.deadline(
                        Commands.waitUntil(feeder::isNotDetected).andThen(Commands.waitSeconds(PIVOT_POST_SHOOT_MOVEMENT_DELAY)),
                        feeder.runWithVoltage(12.0)),
                allowedToFireSupplier,
                xVelocitySupplier,
                yVelocitySupplier,
                angularVelocitySupplier
        );
    }

    public Command level() {
        double[] x = new double[1];
        final double LEVEL_CONSTANT = 0.5;
        return Commands.parallel(
                        Commands.run(() -> x[0] = x[0] + LEVEL_CONSTANT * drivetrain.getRotation().getX() * Robot.defaultPeriodSecs),
                        climber.follow(() -> 0 + Math.max(0.0, x[0]),
                                () -> 0 - Math.min(0.0, x[0])))
                .beforeStarting(() -> x[0] = 0.0);
    }
}
