// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.Idle;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Stormbreaker extends SubsystemBase {
  /** Creates a new Stormbreaker. */

  CANSparkMax storm;
  CANSparkMax breaker;

  SparkPIDController stormPID;

  RelativeEncoder stormCoder;
  RelativeEncoder stormAlternateCoder;

  AnalogPotentiometer pot;

  DigitalInput armLimit;

  

  // shooters will be 6 and 7 vortexs follower
  // DIO 1 for limit switch
  // DIO 0 for beam break sensor
  
  public Stormbreaker() {
    storm = new CANSparkMax(5, MotorType.kBrushless);
    breaker = new CANSparkMax(11, MotorType.kBrushless);

    armLimit = new DigitalInput(1);

    storm.restoreFactoryDefaults();
    breaker.restoreFactoryDefaults();

    stormCoder = storm.getEncoder();
    stormAlternateCoder = storm.getAlternateEncoder(8192);

    stormCoder.setPositionConversionFactor((12/58)*360);
    stormAlternateCoder.setPositionConversionFactor(74.4827586207); // (12/58)*360

    pot = new AnalogPotentiometer(0);

    stormPID = storm.getPIDController();

    storm.setInverted(true);

    breaker.follow(storm, true);

    stormPID.setFeedbackDevice(stormAlternateCoder);

    storm.setIdleMode(IdleMode.kBrake);

    stormCoder.setPosition(0);
    stormAlternateCoder.setPosition(0);

    stormPID.setP(0.062, 0); // was .075
    stormPID.setI(0, 0);
    stormPID.setD(0, 0);
    stormPID.setFF(.002, 0);

    storm.burnFlash();
    breaker.burnFlash();
  }

  public double getPot() {
    return pot.get();
  }

  public boolean getarmLimit() {
    return armLimit.get();
  }

  public double getStormPos() {
    return stormAlternateCoder.getPosition();
  }

  public void summonStormBreaker(double pos) {
    stormPID.setReference(pos, ControlType.kPosition);
  }

  public void speakerPos() {
    stormPID.setReference(10, ControlType.kPosition);
  }

  public double getStormPwr() {
    return storm.getAppliedOutput();
  }

  public double getBreakerPwr() {
    return breaker.getAppliedOutput();
  }

  public void swingStormBreaker(double spd) {
    storm.set(spd*0.12); // was .47
  }

  public boolean atArmSpeaker() {
    return getStormPos() > 9;
  }
  
  public boolean atArmStow() {
    return getStormPos() > Constants.groundPosition-1;
  }

  public void stormBreakerBrakeMode() {
    storm.setIdleMode(IdleMode.kBrake);
    breaker.setIdleMode(IdleMode.kBrake);
  }

  public void stormBreakerCoast() {
    storm.setIdleMode(IdleMode.kCoast);
    breaker.setIdleMode(IdleMode.kCoast);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Storm coder", stormCoder.getPosition());
    SmartDashboard.putNumber("Storm Alt Coder", getStormPos());
    SmartDashboard.putNumber("pot", getPot());
    SmartDashboard.putBoolean("Arm Limit", getarmLimit());
    SmartDashboard.putNumber("Storm pwr", getStormPwr());
    SmartDashboard.putNumber("Breaker pwr", getBreakerPwr());

    if(getarmLimit()) {
      stormCoder.setPosition(0);
      stormAlternateCoder.setPosition(0);
    }
  }
}
