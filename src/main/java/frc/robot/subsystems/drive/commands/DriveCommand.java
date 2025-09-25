package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.MecanumDrivetrain;

public class DriveCommand extends Command {

    private final MecanumDrivetrain m_drive;
    private final Pose2d pow;

    public DriveCommand(MecanumDrivetrain drive, Pose2d pow) {
        m_drive = drive;
        this.pow = pow;
    }

    @Override
    public void execute() {
        m_drive.driveCartesian(pow.getX(), pow.getY(), pow.getRotation().getRadians());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
