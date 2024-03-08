// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Autos.Paths.*;

import java.util.Set;

import javax.xml.crypto.dsig.spec.HMACParameterSpec;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Autos.Drive;
import frc.robot.Autos.Score_Speaker;
import frc.robot.Autos.Score_Trap;
import frc.robot.Autos.Shoot_only;
import frc.robot.Autos.Speaker_Simple;
import frc.robot.commands.*;
// import frc.robot.commands.Elevator_Down;
// import frc.robot.commands.Elevator_Up;
// import frc.robot.commands.Elevator_Toggle;
// import frc.robot.commands.Intake_Feed;
// import frc.robot.commands.Intake_Ground;
// import frc.robot.commands.Intake_Reverse;
// import frc.robot.commands.Shoot_High;
// import frc.robot.commands.Shoot_Slow;

import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.*;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Elevator_Drive;
// import frc.robot.subsystems.Elevator_Tilt;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Shooter;
import pabeles.concurrency.IntRangeConsumer;

// import frc.robot.Auton.AutoChooser;


public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


  /* add the subsytems */
    private final Elevator_Tilt m_elevator_Tilt = new Elevator_Tilt();
    private final Elevator_Drive m_elevator_Drive = new Elevator_Drive();
    private final Intake m_intake = new Intake();
    private final Shooter m_shooter = new Shooter();
   // private final AutoChooser AutoChooser = new AutoChooser();


  /* add the joysticks */
  private final Joystick coDriver = new Joystick(1);
private final XboxController driver = new XboxController(0);

/* smartdashboard buttons */
SendableChooser<Command> autoChooser = new SendableChooser<Command>();


  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    joystick.a().toggleOnTrue(drivetrain.applyRequest(() -> brake));
    //joystick.b().whileTrue(drivetrain
    //    .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);

    joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    // Subsystem Buttons

    final JoystickButton ElevatorToggle = new JoystickButton(driver, XboxController.Button.kX.value);
    ElevatorToggle.onTrue(new Elevator_Toggle(m_elevator_Tilt).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
                            
    // final JoystickButton InakeGround = new JoystickButton(driver, XboxController.Button.kY.value);        
    // InakeGround.toggleOnTrue(new Intake_Ground( m_intake ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // final JoystickButton InakeReverse = new JoystickButton(driver, XboxController.Button.kRightBumper.value);        
    // InakeReverse.whileTrue(new Intake_Reverse( m_intake ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton IntakeFeed = new JoystickButton(driver, XboxController.Button.kB.value);        
    IntakeFeed.toggleOnTrue(new Intake_Feed( m_shooter, m_intake ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
                            
    final JoystickButton ShootFast = new JoystickButton(driver, XboxController.Button.kY.value);        
    ShootFast.onTrue(new Speaker( m_intake, m_shooter ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
                            
    final JoystickButton ShootSlow = new JoystickButton(driver, XboxController.Button.kRightBumper.value);        
    ShootSlow.onTrue(new Amp( m_intake, m_shooter ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
                            
    final JoystickButton ElevatorUp = new JoystickButton(coDriver, 3);        
    ElevatorUp.whileTrue(new Elevator_Up( m_elevator_Drive ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton ElevatorDown = new JoystickButton(coDriver, 4);
    ElevatorDown.whileTrue(new Elevator_Down(m_elevator_Drive).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    }

  public RobotContainer() {
    
    // Set the logger to log to the first flashdrive plugged in
    SignalLogger.setPath("/media/sda1/");
    // Explicitly start the logger
    SignalLogger.start();

    configureBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser =  AutoBuilder.buildAutoChooser("My Default Auto");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // autoChooser.setDefaultOption("Drive", runAuto);
    // autoChooser.addOption("Drive and turn", runAuto);
    // autoChooser.addOption("Silly spin", runAuto);
    // autoChooser.addOption("oneMeter", runAuto);
    // autoChooser.addOption("trap", runAuto );
    autoChooser.setDefaultOption("Drive", new Drive(drivetrain));
    autoChooser.addOption("Trap", new Score_Trap(drivetrain, m_intake, m_shooter, m_elevator_Drive, m_elevator_Tilt) );
    autoChooser.addOption("Speaker_1", new Score_Speaker(drivetrain, m_intake, m_shooter, m_elevator_Drive, m_elevator_Tilt));
    autoChooser.addOption("Speaker_Simple", new Speaker_Simple(drivetrain, m_intake, m_shooter, m_elevator_Drive, m_elevator_Tilt));
    autoChooser.addOption("Speakeronly", new Shoot_only(drivetrain, m_intake, m_shooter, m_elevator_Drive, m_elevator_Tilt));



    // Explicitly stop logging
    // If the user does not call stop(), then it's possible to lose the last few seconds of data
    SignalLogger.stop();
    
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    //return runAuto; // change to lines below when autochoser is figured out
  //return new PathPlannerAuto("Drive"); 
   return autoChooser.getSelected();
  }
}
