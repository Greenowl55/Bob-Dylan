package frc.robot.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import static frc.robot.Autos.Paths.*;

import com.choreo.lib.ChoreoTrajectoryState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CenterKill extends SequentialCommandGroup{
    public CenterKill (CommandSwerveDrivetrain swerve, Intake Intake, Shooter Shooter, Elevator_Drive elevator, Elevator_Tilt rams){

    Pose2d pose = PathPlannerAuto.getStaringPoseFromAutoFile("centerkillauto");

        addCommands(
            Commands.runOnce(() ->  swerve.seedFieldRelative(pose)),

            Intake.runOnce(() -> Intake.Intakerun(-0.2)),
            Shooter.runOnce(() -> Shooter.ShooterRunFront(0.8)), //spin up the shooter
            Shooter.runOnce(()-> Shooter.ShooterRunBack(0.8)),
            Commands.waitSeconds(0.75), // 1.5 sec known good time. seems to work fine at 0.75. can it go lower?
            Intake.runOnce(() -> Intake.Intakerun(1)), // shoot the note
            Commands.waitSeconds(0.5),
            Shooter.runOnce(() -> Shooter.ShooterRunFront(0)),
            Shooter.runOnce(()-> Shooter.ShooterRunBack(0)), // turn off the shooter
            Intake.runOnce(() -> Intake.Intakerun(0)),

            new PathPlannerAuto(("centerkillauto")), //hits all the notes in the center to disrupt ops.
            Commands.waitSeconds(10),
            // shoot the preloaded note after hitting the center notes
            new PathPlannerAuto(("centerkillauto2")), // goes back out to center and alligns for firld orentation
            Commands.waitSeconds(3),
            Commands.runOnce( () -> swerve.seedFieldRelative()) //sets field orrientation?
        );
    }

}