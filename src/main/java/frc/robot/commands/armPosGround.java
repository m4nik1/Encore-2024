// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class armPosGround extends Command {
  /** Creates a new armPosGround. */
  public armPosGround() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.stormBreaker);
    // addRequirements(RobotContainer.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean noteDetect = RobotContainer.intake.detectNote();

    if(!noteDetect) {
      RobotContainer.stormBreaker.summonStormBreaker(Constants.groundPosition);
      RobotContainer.intake.actuateIntake(-.5);
    }
    else {
      RobotContainer.intake.actuateIntake(0);
      // RobotContainer.runRumble(); // feedback to both controllers that the note is in the intake
    //   RobotContainer.stormBreaker.summonStormBreaker(-16);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.intake.actuateIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return RobotContainer.intake.detectNote();
    return false;
  }
}
