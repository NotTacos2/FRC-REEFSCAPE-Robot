package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.Constants.HardwareConstants;
import frc.robot.util.LimelightHelpers;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class MecanumDrivetrain extends SubsystemBase {
    
    private final SparkMax m_frontLeft;
    private final SparkMax m_rearLeft;
    private final SparkMax m_frontRight;
    private final SparkMax m_rearRight;

    private final MecanumDrive m_robotDrive;
    
    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

    public MecanumDrivetrain() {
        m_gyro.zeroYaw();

        m_frontLeft = new SparkMax(HardwareConstants.kFrontLeftMotorID, MotorType.kBrushed);
        m_rearLeft = new SparkMax(HardwareConstants.kRearLeftMotorID, MotorType.kBrushed);
        m_frontRight = new SparkMax(HardwareConstants.kFrontRightMotorID, MotorType.kBrushed);
        m_rearRight = new SparkMax(HardwareConstants.kRearRightMotorID, MotorType.kBrushed);

        SparkMaxConfig inverted = new SparkMaxConfig();
        inverted.inverted(true);
        
        SparkMaxConfig nonInverted = new SparkMaxConfig();
        nonInverted.inverted(false);

        m_frontLeft.configure(inverted, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_rearLeft.configure(inverted, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_rearRight.configure(inverted, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_frontRight.configure(nonInverted, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        m_robotDrive = new MecanumDrive(m_frontLeft::set, m_rearLeft::set, m_frontRight::set, val -> m_rearRight.set(-val));
        
        SmartDashboard.putData(m_gyro);
        SmartDashboard.putData(m_robotDrive);
        SmartDashboard.putData(m_field);


        //TODO: Fix this
        SmartDashboard.putData("Reset Gyro", new InstantCommand(() -> {
            m_gyro.zeroYaw();
            System.out.println("reset");
        }));
    }

    public void driveCartesian(double xSpeed, double ySpeed, double rotSpeed) {
        m_robotDrive.driveCartesian(xSpeed, ySpeed, rotSpeed);
    }

    private double currXSpeed = 0;
    private double currYSpeed = 0;

    public void driveCartesianFieldRelative(double xSpeed, double ySpeed, double rotSpeed) {
        Rotation2d rot = getRotation();
        m_robotDrive.driveCartesian(xSpeed, ySpeed, rotSpeed, rot);

        currXSpeed = Math.max(Math.min(xSpeed, 1), -1);
        currYSpeed = Math.max(Math.min(ySpeed, 1), -1);
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(m_gyro.getYaw());
    }

    public void setPose(Pose2d pose) {
        m_poseEstimate = pose;
        m_field.setRobotPose(pose);
    }

    private final Field2d m_field = new Field2d();
    private Pose2d m_poseEstimate = new Pose2d();


    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation(HardwareConstants.kLimelightName, m_gyro.getYaw(), m_gyro.getRate(),0,0,0,0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(HardwareConstants.kLimelightName);

        if(mt2 != null && mt2.tagCount > 0){
            m_poseEstimate = mt2.pose;
        } else {
            Rotation2d rot = getRotation();

            //Doesn't work for mecanum
            m_poseEstimate = new Pose2d(
                m_poseEstimate.getX() + (Math.abs(rot.getCos()) * HardwareConstants.kXSpeed + Math.abs(rot.getSin()) * HardwareConstants.kYSpeed) * currXSpeed * frc.robot.Constants.dt,
                m_poseEstimate.getY() + (Math.abs(rot.getCos()) * HardwareConstants.kYSpeed + Math.abs(rot.getSin()) * HardwareConstants.kXSpeed) * currYSpeed * frc.robot.Constants.dt,
                rot);
                SmartDashboard.putNumber("Cos", rot.getCos());
                SmartDashboard.putNumber("Sin", rot.getSin());
        }
        
        m_field.setRobotPose(m_poseEstimate);
    }

    @Override
    public void simulationPeriodic() {
        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[4]");
        SimDouble gyroSimAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        gyroSimAngle.set(m_poseEstimate.getRotation().getDegrees());
    }

    public Pose2d getPose() {
        return m_poseEstimate;
    }

    public void reset() {
        currXSpeed = 0;
        currYSpeed = 0;
    }
}
