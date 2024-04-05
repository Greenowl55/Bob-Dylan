package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Elevator_Drive;

public class Climber_Down extends SequentialCommandGroup{

    private final Elevator_Drive Elevator;

    public Climber_Down (Elevator_Drive m_elevator) {

        Elevator = m_elevator;
        addRequirements(m_elevator);

        addCommands(
            Elevator.runOnce(() -> Elevator.Elevatormove(-0.75)),
            Commands.waitSeconds(1.9),
            Elevator.runOnce(() -> Elevator.Elevatormove(0))
            );
      }

}
