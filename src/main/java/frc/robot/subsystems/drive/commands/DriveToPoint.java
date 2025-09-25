package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.MecanumDrivetrain;


//DOES NOT DRIVE TO POINT
public class DriveToPoint extends Command {

    private final MecanumDrivetrain m_drive;
    private final Pose2d target;

    private final PIDController headingController = new PIDController(0.045, 0, 0);

    public DriveToPoint(MecanumDrivetrain drive, Pose2d point) {
        m_drive = drive;
        target = point;

        SmartDashboard.putData("Heading PID", headingController);
    }

    @Override
    public void execute() {
        Pose2d currPose = m_drive.getPose();
        Pose2d diff = new Pose2d(target.getX() - currPose.getX(), target.getY() - currPose.getY(), target.getRotation().minus(currPose.getRotation()));

        double angle = diff.getRotation().getDegrees();
        if(angle > 180) angle = 360 - angle;
        double anglePow = headingController.calculate(m_drive.getRotation().getDegrees(), 0);
    
        m_drive.driveCartesian(target.getX(), target.getY(), anglePow);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
}
