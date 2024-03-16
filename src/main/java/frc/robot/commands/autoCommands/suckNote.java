// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class suckNote extends Command {
  /** Creates a new suckNote. */
  Timer timer = new Timer();

  boolean finished = false;
  public suckNote() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
    addRequirements(RobotContainer.stormBreaker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    timer.reset();
    RobotContainer.stormBreaker.summonStormBreaker(Constants.autoGroundPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.stormBreaker.summonStormBreaker(Constants.autoGroundPos);
    timer.start();
    if(!RobotContainer.intake.detectNote()) {
      RobotContainer.intake.actuateIntake(-.5);
    }
    else {
      RobotContainer.intake.actuateIntake(0);
      // finished = true;
    }

    if(timer.hasElapsed(2)) {
      RobotContainer.intake.actuateIntake(0);
      finished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.actuateIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
