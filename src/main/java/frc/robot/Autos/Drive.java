package frc.robot.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;
import frc.robot.Robot;
import frc.robot.commands.*;
import static frc.robot.Autos.Paths.*;

import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.*;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Drive extends SequentialCommandGroup{

    public Drive (CommandSwerveDrivetrain swerve){


        Pose2d pose = PathPlannerAuto.getStaringPoseFromAutoFile("Drive1");
        // var alliance = DriverStation.getAlliance();
        //     if (alliance.isPresent()) {
        //         return alliance.get() == DriverStation.Alliance.Red;
        //     }
        //     return false;
        //Pose2d pose = new Pose2d(1.393, 5.459, Rotation2d.fromRadians(3.142));

        //swerve.runOnce(() -> swerve.seedFieldRelative())
        addCommands(
            //Commands.runOnce(() -> Robot.m_pigieon2.setYaw(0)),
            Commands.runOnce(() ->  swerve.seedFieldRelative(pose)),
            new WaitCommand(1),
            //AutoBuilder.followPath(Drive)
            new PathPlannerAuto(("Drive1")) // this would work but only with pathplanner autos

        );
    }

}
