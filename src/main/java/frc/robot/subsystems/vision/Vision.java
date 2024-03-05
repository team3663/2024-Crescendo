package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.*;
import java.util.function.Consumer;

public class Vision extends SubsystemBase {
    private static final double MAXIMUM_ACCEPTED_MEASUREMENT_Z = 1.0;

    private static final InterpolatingMatrixTreeMap<Double, N3, N1> MEASUREMENT_STD_DEV_DISTANCE_MAP = new InterpolatingMatrixTreeMap<>();

    private final VisionIo[] ios;
    private final VisionInputsAutoLogged[] visionInputs;

    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    private List<VisionMeasurement> acceptedMeasurements = Collections.emptyList();

    static {
        MEASUREMENT_STD_DEV_DISTANCE_MAP.put(1.0, VecBuilder.fill(2.0, 2.0, 2.0));
        MEASUREMENT_STD_DEV_DISTANCE_MAP.put(4.0, VecBuilder.fill(10.0, 10.0, 10.0));
    }

    public Vision(VisionIo[] ios) {
        this.ios = ios;

        visionInputs = new VisionInputsAutoLogged[this.ios.length];
        for (int i = 0; i < visionInputs.length; i++) {
            visionInputs[i] = new VisionInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(visionInputs[i]);
            Logger.processInputs("Vision/" + ios[i].getConstants().name(), visionInputs[i]);
        }

        List<VisionMeasurement> acceptedMeasurements = new ArrayList<>();
        for (VisionInputsAutoLogged visionInput : visionInputs) {
            Pose3d pose = visionInput.estimatedPose;
            double timestamp = visionInput.timestampSeconds;

            // Skip inputs that haven't updated
            if (!visionInput.poseUpdated) continue;

            // Skip measurements that are not with in the field boundary
            if (pose.getX() < 0.0 || pose.getX() > fieldLayout.getFieldLength() ||
                    pose.getY() < 0.0 || pose.getY() > fieldLayout.getFieldWidth())
                continue;

            // Skip measurements that are floating
            if (pose.getZ() > MAXIMUM_ACCEPTED_MEASUREMENT_Z) continue;

            // Compute the standard deviation to use based on the distance to the closest tag
            OptionalDouble closestTagDistance = Arrays.stream(visionInput.targetIds)
                    .mapToObj(fieldLayout::getTagPose)
                    .filter(Optional::isPresent)
                    .mapToDouble(v -> v.get().getTranslation().getDistance(pose.getTranslation()))
                    .min();
            // If for some reason we were unable to calculate the distance to the closest tag, assume we are infinitely far away
            Matrix<N3, N1> stdDevs = MEASUREMENT_STD_DEV_DISTANCE_MAP.get(closestTagDistance.orElse(Double.MAX_VALUE));

            acceptedMeasurements.add(new VisionMeasurement(timestamp, pose.toPose2d(), stdDevs));
        }
        this.acceptedMeasurements = acceptedMeasurements;
    }

    /**
     * @return List of updated vision measurements to be passed to drivetrain.
     */
    public List<VisionMeasurement> getVisionMeasurements() {
        return acceptedMeasurements;
    }

    /**
     * @return Command that consumes vision measurements
     */
    public Command consumeVisionMeasurements(Consumer<List<VisionMeasurement>> visionMeasurementConsumer) {
        return Commands.run(() -> visionMeasurementConsumer.accept(acceptedMeasurements));
    }
}
