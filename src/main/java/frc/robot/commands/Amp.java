package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
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


public class Amp extends SequentialCommandGroup{
    private final Shooter shooter;
    private final Intake intake;
    private final Shoot_Slow shooterslow;
    private final Intake_Feed intakerun;

    public Amp (Intake intake, Shooter m_shooter) {

        shooter = m_shooter;
        addRequirements(m_shooter);

        addCommands(

        // new ParallelCommandGroup(
        //     new RunCommand(shooterslow, m_shooter)    
        //         new SequentialCommandGroup(
        //             new WaitCommand(1)
        //             new RunCommand(null, null)
                    
        //         )

        // );
                    parrallel(shooterslow.withTimeout(1)),
                        SequenceInputStream(new WaitCommand(0.5),
                                 (intake).withTimeout(1)
                        )

        );
    }

}
