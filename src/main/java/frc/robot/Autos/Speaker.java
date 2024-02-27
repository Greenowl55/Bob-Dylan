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

public class Speaker extends SequentialCommandGroup{
    public Speaker (CommandSwerveDrivetrain swerve, Intake Intake, Shooter Shooter, Elevator_Drive elevator, Elevator_Tilt rams){

        addCommands(
            Shooter.runOnce(() -> Shooter.ShooterRunFront(0.99)),
            Shooter.runOnce(()-> Shooter.ShooterRunBack(0.99)),
            Commands.waitSeconds(0.5),
            Intake.runOnce(() -> Intake.Intakerun(0.4) ),
            Commands.waitSeconds(1),
            Shooter.runOnce(() -> Shooter.ShooterRunFront(0)),
            Shooter.runOnce(()-> Shooter.ShooterRunBack(0)),
            Intake.runOnce(() -> Intake.Intakerun(0) ),
            AutoBuilder.followPath(moveandrotate)
        );
    }

}