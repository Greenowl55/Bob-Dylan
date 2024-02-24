package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.*;
import edu.wpi.first.wpilibj.*;

import java.io.SequenceInputStream;
import java.util.function.DoubleSupplier;

import frc.robot.commands.*;
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
            shooter.runOnce(() -> shooter.ShooterRun(0.99)),
            Commands.waitSeconds(2),
            intake.runOnce(() -> intake.Intakerun(0.5)),
            Commands.waitSeconds(0.5),
            m_shooter.runOnce(() -> m_shooter.ShooterRun(0)),
            intake.runOnce(() -> intake.Intakerun(0))
        );
    }

}
