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

        addCommands(
            Commands.runOnce(() ->  swerve.seedFieldRelative(pose)),
            new WaitCommand(1),
            new PathPlannerAuto(("Drive1"))

        );
    }

}
