// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Autos.*;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;


public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed for musky days set to " private double MaxSpeed = 2; "
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

    final JoystickButton ClimberUp = new JoystickButton(coDriver, 1);        
    ClimberUp.onTrue(new Climber_Up( m_elevator_Drive ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton ClimberDown = new JoystickButton(coDriver, 2);        
    ClimberDown.onTrue(new Climber_Down( m_elevator_Drive ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

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
    

    autoChooser.setDefaultOption("Drive", new Drive(drivetrain));
    // autoChooser.addOption("Trap", new Score_Trap(drivetrain, m_intake, m_shooter, m_elevator_Drive, m_elevator_Tilt) );
    autoChooser.addOption("Speaker_Feed_Side", new Speaker_Feed_Side(drivetrain, m_intake, m_shooter, m_elevator_Drive, m_elevator_Tilt));
    autoChooser.addOption("Speaker_Amp_Side", new Speaker_Amp_Side(drivetrain, m_intake, m_shooter, m_elevator_Drive, m_elevator_Tilt));
    autoChooser.addOption("Speaker_Center", new Speaker_Center(drivetrain, m_intake, m_shooter, m_elevator_Drive, m_elevator_Tilt));
    autoChooser.addOption("Canter and Drive", new SimpleCenter(drivetrain, m_intake, m_shooter, m_elevator_Drive, m_elevator_Tilt));
    autoChooser.addOption("Speakeronly", new Shoot_only(drivetrain, m_intake, m_shooter, m_elevator_Drive, m_elevator_Tilt));
    autoChooser.addOption("FeedSide_Centernote", new CenterKill(drivetrain, m_intake, m_shooter, m_elevator_Drive, m_elevator_Tilt));




    // Explicitly stop logging
    // If the user does not call stop(), then it's possible to lose the last few seconds of data
    SignalLogger.stop();
    
  }

  public Command getAutonomousCommand() {

   return autoChooser.getSelected();

  }
}
