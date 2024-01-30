package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.*;

import static edu.wpi.first.units.Units.*;

public class SimpleSimDrivetrain implements DrivetrainIO {
    private final Measure<Time> updatePeriod;

    private final PIDController rotationController = new PIDController(10, 0.0, 0.0);

    private final MutableMeasure<Distance> x = MutableMeasure.zero(Meters);
    private final MutableMeasure<Velocity<Distance>> xVelocity = MutableMeasure.zero(MetersPerSecond);
    private final MutableMeasure<Distance> y = MutableMeasure.zero(Meters);
    private final MutableMeasure<Velocity<Distance>> yVelocity = MutableMeasure.zero(MetersPerSecond);
    private final MutableMeasure<Angle> rotation = MutableMeasure.zero(Radians);
    private final MutableMeasure<Velocity<Angle>> rotationalVelocity = MutableMeasure.zero(RadiansPerSecond);
    private Rotation2d targetAngle = null;


    public SimpleSimDrivetrain() {
        this(Milliseconds.of(20));
    }

    public SimpleSimDrivetrain(Measure<Time> updatePeriod) {
        this.updatePeriod = updatePeriod;

        rotationController.enableContinuousInput(0.0, 2.0 * Math.PI);
    }

    @Override
    public void updateInputs(Inputs inputs) {
        x.mut_plus((Measure<Distance>) xVelocity.times(updatePeriod));
        y.mut_plus((Measure<Distance>) yVelocity.times(updatePeriod));
        if (targetAngle != null) {
            rotationalVelocity.mut_replace(
                    rotationController.calculate(rotation.in(Radians), targetAngle.getRadians()),
                    RadiansPerSecond
            );
        }

        rotation.mut_plus((Measure<Angle>) rotationalVelocity.times(updatePeriod));

        inputs.pose = new Pose2d(x.in(Meters), y.in(Meters), Rotation2d.fromRadians(rotation.in(Radians)));
        inputs.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity,
                yVelocity,
                rotationalVelocity,
                inputs.pose.getRotation()
        );
    }

    @Override
    public void drive(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                chassisSpeeds,
                Rotation2d.fromRadians(rotation.in(Radians))
        );

        driveFieldOriented(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond, fieldRelativeSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void driveFieldOriented(double xVelocity, double yVelocity, double rotationalVelocity) {
        this.xVelocity.mut_replace(xVelocity, MetersPerSecond);
        this.yVelocity.mut_replace(yVelocity, MetersPerSecond);
        this.rotationalVelocity.mut_replace(rotationalVelocity, RadiansPerSecond);
        this.targetAngle = null;
    }

    @Override
    public void driveFieldOrientedFacingAngle(double xVelocity, double yVelocity, Rotation2d angle) {
        this.xVelocity.mut_replace(xVelocity, MetersPerSecond);
        this.yVelocity.mut_replace(yVelocity, MetersPerSecond);
        targetAngle = angle;
    }

    @Override
    public void stop() {
        xVelocity.mut_replace(0, MetersPerSecond);
        yVelocity.mut_replace(0, MetersPerSecond);
        rotationalVelocity.mut_replace(0, RadiansPerSecond);
        targetAngle = null;
    }

    @Override
    public HolonomicPathFollowerConfig getPathFollowerConfig() {
        return new HolonomicPathFollowerConfig(
                new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(1.0, 0.0, 0.0), // Rotation PID constants
                4.5, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig()
        );
    }

    @Override
    public void resetPose(Pose2d pose) {
        x.mut_plus(pose.getX(), Meters);
        y.mut_plus(pose.getY(), Meters);
        rotation.mut_plus(pose.getRotation().getRadians(), Radians);


    }

    @Override
    public void zeroGyroscope() {
        rotation.mut_replace(0, Radians);
    }
}
