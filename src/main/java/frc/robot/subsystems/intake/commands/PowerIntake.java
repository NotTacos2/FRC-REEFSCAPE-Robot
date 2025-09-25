package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class PowerIntake extends Command {

    private final Intake m_intake;
    private final double m_pow;

    public PowerIntake(Intake intake, double pow) {
        m_intake = intake;
        m_pow = pow;
    }

    @Override
    public void execute() {
        m_intake.setMotor(m_pow);
    }
}
