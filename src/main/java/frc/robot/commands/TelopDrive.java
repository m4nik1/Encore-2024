// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TelopDrive extends Command {
  /** Creates a new TelopDrive. */
  SlewRateLimiter rotationLimiter, translateLimiter, strafeLimiter;

  public TelopDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrain);

    rotationLimiter = new SlewRateLimiter(3.0);
    translateLimiter = new SlewRateLimiter(3.0);
    strafeLimiter = new SlewRateLimiter(3.0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedMultiplier = 0.72;
    double getX = RobotContainer.getLeftX();
    double getY = RobotContainer.getLeftY();
    double getRotation = RobotContainer.getRightX();

    double translationVal = translateLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(-getY, .05));
    double strafeVal = strafeLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(-getX, .05));
    double rotationVal = rotationLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(-getRotation, .04));

    Translation2d translation = new Translation2d(translationVal, strafeVal);

    RobotContainer.driveTrain.drive(translation.times(Constants.maxSpeed), rotationVal * Constants.maxAngularSpd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
