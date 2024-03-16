// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.armPosAmp;
import frc.robot.commands.armPosGround;
import frc.robot.commands.armPosSpeaker;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.TelopDrive;
import frc.robot.commands.runIntake;
import frc.robot.commands.ampScore;
import frc.robot.commands.armPosStow;
import frc.robot.commands.climb;
import frc.robot.commands.climbDown;
import frc.robot.commands.revShooter;
import frc.robot.commands.swingStormBreaker;
import frc.robot.commands.autoCommands.shootNote;
import frc.robot.commands.autoCommands.stowPosAuto;
import frc.robot.commands.autoCommands.suckNote;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Stormbreaker;
import frc.robot.subsystems.climber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private SendableChooser<Command> autoChooser;

  public static DriveTrain driveTrain;
  public static Stormbreaker stormBreaker;
  public static Intake intake;
  public static Shooter shooter;
  public static climber Climber;
  
  static CommandXboxController driverController;
  static CommandXboxController operatorController;

  private final Field2d field;

  // private void configureEncoreAuto()  {
  //   NamedCommands.registerCommand("Shoot", new shoot().withTimeout(.5));
  // }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    driveTrain = new DriveTrain();
    stormBreaker = new Stormbreaker();
    intake = new Intake();
    shooter = new Shooter();
    Climber = new climber();

    NamedCommands.registerCommand("Shoot Note", new shootNote());
    NamedCommands.registerCommand("Obtain Note", new suckNote());
    NamedCommands.registerCommand("Stow Arm", new stowPosAuto());
    NamedCommands.registerCommand("Speaker arm", new armPosSpeaker());

    field = new Field2d();

    SmartDashboard.putData("Field", field);

    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      field.setRobotPose(pose);
    });

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      field.getObject("target pose").setPose(pose);
    });

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      field.getObject("path").setPoses(poses);
    });

    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);


    driveTrain.setDefaultCommand(new TelopDrive());
    intake.setDefaultCommand(new runIntake());
    stormBreaker.setDefaultCommand(new swingStormBreaker());

    configureBindings();

    // autoChooser.reset();
    
    autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public static double getLeftX() {
    return driverController.getLeftX();
  }

  public static double getLeftY() {
    return driverController.getLeftY();
  }

  public static double getRightX() {
    return driverController.getRightX();
  }

  public static double getLeftYOp() {
    return operatorController.getLeftY();
  }

  public static double getRightYOp() {
    return operatorController.getRightY();
  }

  public static void runRumble() {
    operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, .5);
    // driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, .5);
  }

  public static void stopRumble() {
    operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
    // driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
  }

  private void configureBindings() {
    operatorController.y().onTrue(new armPosAmp());
    operatorController.povDown().whileTrue(new swingStormBreaker());
    operatorController.x().onTrue(new armPosStow());
    operatorController.rightBumper().whileTrue(new ampScore());
    operatorController.a().onTrue(new armPosGround());


    operatorController.leftTrigger().whileTrue(new revShooter());
    operatorController.b().onTrue(new armPosSpeaker());
    operatorController.rightTrigger().whileTrue(new IntakeIn());

    // driverController.povDown().whileTrue(new climb());
    // driverController.povUp().whileTrue(new climbDown());


    driverController.rightBumper().onTrue(new InstantCommand(() -> driveTrain.resetGyro()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
