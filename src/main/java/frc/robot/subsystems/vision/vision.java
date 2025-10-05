package frc.robot.subsystems.vision;

import frc.robot.Constants.HardwareConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;

public class vision extends SubsystemBase {
    LimelightResults results = LimelightHelpers.getLatestResults(HardwareConstants.kLimelightName);

    public vision() {
        LimelightHelpers.setPipelineIndex(HardwareConstants.kLimelightName, 0);
    }

    public int getAprilTagID() {
        if (results.valid) {
            if (results.targets_Fiducials.length > 0) {
                LimelightTarget_Fiducial tag = results.targets_Fiducials[0];
                return tag.ID;
            }
        }
        return 0;
    }
}
