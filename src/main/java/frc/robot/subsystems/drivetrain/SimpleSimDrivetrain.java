package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.*;

import static edu.wpi.first.units.Units.*;

public class SimpleSimDrivetrain implements DrivetrainIO {
    private final Measure<Time> updatePeriod;

    private final MutableMeasure<Distance> x = MutableMeasure.zero(Meters);
    private final MutableMeasure<Velocity<Distance>> xVelocity = MutableMeasure.zero(MetersPerSecond);
    private final MutableMeasure<Distance> y = MutableMeasure.zero(Meters);
    private final MutableMeasure<Velocity<Distance>> yVelocity = MutableMeasure.zero(MetersPerSecond);
    private final MutableMeasure<Angle> rotation = MutableMeasure.zero(Radians);
    private final MutableMeasure<Velocity<Angle>> rotationalVelocity = MutableMeasure.zero(RadiansPerSecond);

    public SimpleSimDrivetrain() {
        this(Milliseconds.of(20));
    }

    public SimpleSimDrivetrain(Measure<Time> updatePeriod) {
        this.updatePeriod = updatePeriod;
    }

    @Override
    public void updateInputs(Inputs inputs) {
        x.mut_plus((Measure<Distance>) xVelocity.times(updatePeriod));
        y.mut_plus((Measure<Distance>) yVelocity.times(updatePeriod));
        rotation.mut_plus((Measure<Angle>) rotationalVelocity.times(updatePeriod));

        inputs.pose = new Pose2d(x.in(Meters), y.in(Meters), Rotation2d.fromRadians(rotation.in(Radians)));
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
    }

    @Override
    public void stop() {
        xVelocity.mut_replace(0, MetersPerSecond);
        yVelocity.mut_replace(0, MetersPerSecond);
        rotationalVelocity.mut_replace(0, RadiansPerSecond);
    }

    @Override
    public void zeroGyroscope() {
        rotation.mut_replace(0, Radians);
    }
}
