package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;

public class Intake extends SubsystemBase {

    SparkMax m_motor = new SparkMax(HardwareConstants.kIntakeMotorID, MotorType.kBrushed);

    public Intake() {}

    public void setMotor(double pow) {
        m_motor.set(pow);
    }
}
