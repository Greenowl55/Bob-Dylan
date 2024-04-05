package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Speaker extends SequentialCommandGroup{
    private final Shooter shooter;
    private final Intake intake;

    public Speaker (Intake m_intake, Shooter m_shooter) {

        shooter = m_shooter;
        intake = m_intake;
        addRequirements(m_shooter);

        addCommands(
            intake.runOnce(() -> intake.Intakerun(-0.2)),
            shooter.runOnce(() -> shooter.ShooterRunFront(1)),
            shooter.runOnce(()-> shooter.ShooterRunBack(1)),
            Commands.waitSeconds(0.75), // 1.5 sec known good time. seems to work fine at 0.75. can it go lower?
            intake.runOnce(() -> intake.Intakerun(1)),
            Commands.waitSeconds(0.5),
            shooter.runOnce(() -> shooter.ShooterRunFront(0)),
            shooter.runOnce(()-> shooter.ShooterRunBack(0)),
            intake.runOnce(() -> intake.Intakerun(0))
        );
    }

}