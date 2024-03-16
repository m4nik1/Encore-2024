// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import java.sql.Time;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class shootNote extends Command {
  /** Creates a new shootNote. */
  boolean finished = false;
  Timer timer = new Timer();

  public shootNote() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
    addRequirements(RobotContainer.shooter);
    addRequirements(RobotContainer.stormBreaker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    finished = false;
    RobotContainer.shooter.startShooter();
    RobotContainer.stormBreaker.summonStormBreaker(Constants.speakerShot);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finished = false;
    RobotContainer.stormBreaker.summonStormBreaker(Constants.speakerShot);
    timer.start();

    if(RobotContainer.shooter.isAtSpeed(3100) && timer.hasElapsed(2)) {
      RobotContainer.intake.runIndexer();
    }
    SmartDashboard.putNumber("Timer", timer.get());
    if(timer.hasElapsed(3)) {
      RobotContainer.shooter.stopShooter();
      RobotContainer.intake.actuateIntake(0);
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.stopShooter();
    RobotContainer.intake.actuateIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}