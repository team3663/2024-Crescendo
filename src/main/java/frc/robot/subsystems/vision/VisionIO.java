package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {
    void updateInputs(Inputs inputs);

    class Inputs implements LoggableInputs {
        public double tagYawRad = 0;
        public boolean tagFound = false;
        public int tagID = 0;

        @Override
        public void toLog(LogTable table) {
            table.put("TagFound", tagFound);
            table.put("TagYawAngle", tagYawRad);
            table.put("TagID", tagID);
        }

        @Override
        public void fromLog(LogTable table) {
            tagFound = table.get("TagFound", false);
            tagYawRad = table.get("TagYawAngle", tagYawRad);
            tagID = table.get("TagID", tagID);
        }
    }
}
