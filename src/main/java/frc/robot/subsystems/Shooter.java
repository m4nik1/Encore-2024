// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  
  CANSparkFlex shooter1;
  CANSparkFlex shooter2;

  RelativeEncoder shooter1Encoder;
  RelativeEncoder shooter2Encoder;
  SparkPIDController shooter1Controller;
  SparkPIDController shooter2Controller;
  

  public Shooter() {
    shooter1 = new CANSparkFlex(6, MotorType.kBrushless);
    shooter2 = new CANSparkFlex(7, MotorType.kBrushless);

    shooter1Encoder = shooter1.getEncoder();
    shooter2Encoder = shooter2.getEncoder();

    shooter1Controller = shooter1.getPIDController();
    shooter2Controller = shooter2.getPIDController();

    shooter1Controller.setP(0, 0);
    shooter1Controller.setI(0, 0);
    shooter1Controller.setD(0, 0);
    shooter1Controller.setFF(0, 0);

    shooter2.follow(shooter1);
  }

  public void runShooterTest() {
    shooter1.set(-.3);
  }

  public void startShooter() {
    shooter1.set(-.6);
  }

  public boolean isAtSpeed(double rpm) {
    return (shooter1Encoder.getVelocity()) < -rpm && (shooter2Encoder.getVelocity()) < -rpm;
  }

    public void stopShooter() {
    shooter1.set(0);
  }
  
  // public double getShooter() {
  //   return shooter1.getPos`();
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter1 Vel", shooter1Encoder.getVelocity());
    SmartDashboard.putNumber("Shooter2 Vel", shooter1Encoder.getVelocity());


  }
}
