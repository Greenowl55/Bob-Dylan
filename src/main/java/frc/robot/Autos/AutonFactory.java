package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import static frc.robot.Autos.Paths.*;

import com.pathplanner.lib.auto.AutoBuilder;


public class AutonFactory {
    private Shooter shooter;
    private Intake intake;
    private Elevator_Drive elevator_Drive;
    private Elevator_Tilt elevator_Tilt;
    private Shoot_High shoot_High;



    public Command oneMeter(CommandSwerveDrivetrain swerve) {
		return AutoBuilder.followPath(move);
	}

    public Command Trap(CommandSwerveDrivetrain m_swerve){
        return AutoBuilder.followPath(trap);
    }

    public Command Sillyspin(CommandSwerveDrivetrain swerve) {
        return AutoBuilder.followPath(spillyspin);
    }

    public Command MoveandRotate(CommandSwerveDrivetrain swerve) {
        return AutoBuilder.followPath(moveandrotate);
    }
}
