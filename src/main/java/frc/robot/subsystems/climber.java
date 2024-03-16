// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.climb;

public class climber extends SubsystemBase {
  /** Creates a new climber. */
  TalonFX climberMotor;
  DigitalInput climblimitSwitch;

  public climber() {
    climberMotor = new TalonFX(9);
    climblimitSwitch=new DigitalInput(2);
    climberMotor.setInverted(true);
    climberMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void runClimber(double spd) {
    climberMotor.set(spd * .65);
  }
  public boolean getLimit(){
    return !climblimitSwitch.get();
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("limit switch", getLimit());
    if(getLimit()){
      climberMotor.set(0);
    }
  }
}
