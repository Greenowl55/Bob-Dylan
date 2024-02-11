// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Elevator_Down;
import frc.robot.commands.Elevator_Hight;
import frc.robot.commands.Elevator_Toggle;
import frc.robot.commands.Elevator_Up;
import frc.robot.commands.Intake_Feed;
import frc.robot.commands.Intake_Ground;
import frc.robot.commands.Shoot_High;
import frc.robot.commands.Shoot_Slow;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator_Drive;
import frc.robot.subsystems.Elevator_Tilt;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


  /* make some subsytems */
    public final Elevator_Tilt m_elevator_Tilt = new Elevator_Tilt();
    public final Elevator_Drive m_elevator_Drive = new Elevator_Drive();
    public final Intake m_intake = new Intake();
    public final Shooter m_shooter = new Shooter();

  /* make some joysticks */
  private final XboxController coDriver = new XboxController(1);
private final XboxController driver = new XboxController(0);

/* smartdashboard buttons */

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

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
                            
    final JoystickButton inakeGround = new JoystickButton(driver, XboxController.Button.kRightBumper.value);        
    inakeGround.whileTrue(new Intake_Ground( m_intake ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
                            
    final JoystickButton intakeFeed = new JoystickButton(driver, XboxController.Button.kX.value);        
    intakeFeed.whileTrue(new Intake_Feed( m_shooter ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
                            
    final JoystickButton shootFast = new JoystickButton(coDriver, XboxController.Button.kA.value);        
    shootFast.whileTrue(new Shoot_High( m_shooter ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
                            
    final JoystickButton shootSlow = new JoystickButton(coDriver, XboxController.Button.kB.value);        
    shootSlow.whileTrue(new Shoot_Slow( m_shooter ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
                            
    final JoystickButton elevatorHight = new JoystickButton(coDriver, XboxController.Button.kLeftStick.value);        
    elevatorHight.whileTrue(new Elevator_Hight( m_elevator_Drive ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    }

  public RobotContainer() {
    configureBindings();
    
        // Build an auto chooser. This will use Commands.none() as the default option.
    //autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    //SmartDashboard.putData("Auto Chooser", autoChooser);
    
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    //return runAuto; // change to lines below when autochoser is figured out
    return new PathPlannerAuto("Tests"); 
    //return autoChooser.getSelected();
  }
}
