package frc.robot.subsystems.vision;

import frc.robot.Constants.HardwareConstants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;

public class VisionSubsystem extends SubsystemBase {
    LimelightResults results = LimelightHelpers.getLatestResults(HardwareConstants.kLimelightName);

    public VisionSubsystem() {
        LimelightHelpers.setPipelineIndex(HardwareConstants.kLimelightName, 0);
    }

    public double getAprilTagID() { // why is it double for april tag ids?????
        if (results.valid) {
            if (results.targets_Fiducials.length > 0) {
                LimelightTarget_Fiducial tag = results.targets_Fiducials[0];
                return tag.fiducialID; // maybe idk 
            }
        }
        return 0;
    }

    public double getHorizontalAngle() {
        if (results.valid) {
            if (results.targets_Fiducials.length > 0) {
                LimelightTarget_Fiducial tag = results.targets_Fiducials[0];
                return tag.tx; // maybe idk 
            }
        }
        return 0;
    }

    public double getVerticalAngle() {
        if (results.valid) {
            if (results.targets_Fiducials.length > 0) {
                LimelightTarget_Fiducial tag = results.targets_Fiducials[0];
                return tag.ty; // maybe idk 
            }
        }
        return 0;
    }

    public double getTargetArea() {
        if (results.valid) {
            if (results.targets_Fiducials.length > 0) {
                LimelightTarget_Fiducial tag = results.targets_Fiducials[0];
                return tag.ta; // maybe idk 
            }
        }
        return 0;
    }

    public Pose3d getBotPose() {
        Pose3d pose = new Pose3d();
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            pose = LimelightHelpers.getBotPose3d_wpiRed(HardwareConstants.kLimelightName);
        } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            pose = LimelightHelpers.getBotPose3d_wpiBlue(HardwareConstants.kLimelightName);
        }
        if (pose == null) {
            DriverStation.reportWarning("Couldn't detect any pose...", false);
        }
        return pose;
    }
}
