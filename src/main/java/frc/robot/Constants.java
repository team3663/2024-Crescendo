package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utility.SubwooferRotations;

import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final int TEST_CONTROLLER_PORT = 2;

    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    private static final Rotation2d BLUE_AMP_ROTATION = Rotation2d.fromDegrees(-90.0);
    private static final Rotation2d RED_AMP_ROTATION = Rotation2d.fromDegrees(-90.0);

    private static final Translation2d BLUE_SPEAKER_POSITION = FIELD_LAYOUT.getTagPose(7).orElse(new Pose3d()).toPose2d().getTranslation();
    private static final Translation2d RED_SPEAKER_POSITION = FIELD_LAYOUT.getTagPose(4).orElse(new Pose3d()).toPose2d().getTranslation();

    private static final SubwooferRotations BLUE_SUBWOOFER_ROTATIONS = new SubwooferRotations(Rotation2d.fromDegrees(180.0),
            Rotation2d.fromDegrees(-120.0), Rotation2d.fromDegrees(120.0));
    private static final SubwooferRotations RED_SUBWOOFER_ROTATIONS = new SubwooferRotations(Rotation2d.fromDegrees(0.0),
            Rotation2d.fromDegrees(-60.0), Rotation2d.fromDegrees(60.0));

    // If true code will turn on various test feature like enabling and biding the test controller.
    public static boolean ENABLE_TEST_FEATURES = true;

    public static Rotation2d getAmpRotationForAlliance(DriverStation.Alliance alliance) {
        return switch (alliance) {
            case Red -> Constants.RED_AMP_ROTATION;
            case Blue -> Constants.BLUE_AMP_ROTATION;
        };
    }

    public static Translation2d getSpeakerPositionForAlliance(DriverStation.Alliance alliance) {
        return switch (alliance) {
            case Red -> Constants.RED_SPEAKER_POSITION;
            case Blue -> Constants.BLUE_SPEAKER_POSITION;
        };
    }

    public static SubwooferRotations getSubwooferRotationsForAlliance(DriverStation.Alliance alliance) {
        return switch (alliance) {
            case Red -> Constants.RED_SUBWOOFER_ROTATIONS;
            case Blue -> Constants.BLUE_SUBWOOFER_ROTATIONS;
        };
    }
}
