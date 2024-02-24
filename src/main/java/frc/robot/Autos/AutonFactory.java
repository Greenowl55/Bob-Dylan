package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import static frc.robot.Autos.Paths.*;

import com.pathplanner.lib.auto.AutoBuilder;


public class AutonFactory {

    public static Command oneMeter(CommandSwerveDrivetrain swerve) {
		return AutoBuilder.followPath(move);
	}

    public Command Trap(CommandSwerveDrivetrain m_swerve, Shooter m_shooter){
        return AutoBuilder.followPath(trap);
        //m_shooter.ShooterRun(0.99);
    }

    public Command Sillyspin(CommandSwerveDrivetrain swerve) {
        return AutoBuilder.followPath(spillyspin);
    }

    public Command MoveandRotate(CommandSwerveDrivetrain swerve) {
        return AutoBuilder.followPath(moveandrotate);
    }
}
