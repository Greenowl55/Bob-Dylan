package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import static frc.robot.Autos.Paths.*;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Score_Trap extends SequentialCommandGroup{
    public Score_Trap (CommandSwerveDrivetrain swerve, Intake Intake, Shooter Shooter, Elevator_Drive elevator, Elevator_Tilt rams){

        addCommands(
            AutoBuilder.followPath(trap),
            Shooter.runOnce(() -> Shooter.ShooterRun(0.99)),
            Commands.waitSeconds(0.5),
            Intake.runOnce(() -> Intake.Intakerun(0.4) ),
            Commands.waitSeconds(1),
            Shooter.runOnce(() -> Shooter.ShooterRun(0)),
            Intake.runOnce(() -> Intake.Intakerun(0) )
        );
    }

}
