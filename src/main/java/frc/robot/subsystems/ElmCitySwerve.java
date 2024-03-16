// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.lib.math.OnboardModuleState;
import frc.robot.utils.nacConfig;

public class ElmCitySwerve extends SubsystemBase {
  /** Creates a new ElmCitySwerve. */ // climber is on 9
  CANSparkFlex driveMotor;
  CANSparkMax angleMotor;

  RelativeEncoder driveEncoder;
  RelativeEncoder integratedEncoder;
  CANcoder nac;

  // PID Controllers
  SparkPIDController driveController;
  SparkPIDController angleController; 

  // Misc variables
  Rotation2d lastAngle;
  public int moduleNum;

  Rotation2d offset;


  public ElmCitySwerve(int moduleNum, int driveNum, int angleNum, int nacID, Rotation2d angleOffset, boolean driveInvert, boolean angleInvert) {
    this.moduleNum = moduleNum;
    this.offset = angleOffset;

    nac = new CANcoder(nacID);

    driveMotor = new CANSparkFlex(driveNum, MotorType.kBrushless);
    driveMotor.restoreFactoryDefaults();
    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPosition(0);
    driveMotor.setOpenLoopRampRate(0.2);
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveController = driveMotor.getPIDController();

    angleMotor = new CANSparkMax(angleNum, MotorType.kBrushless );
    angleMotor.restoreFactoryDefaults();
    angleMotor.setIdleMode(IdleMode.kCoast);
    angleController = angleMotor.getPIDController();
    integratedEncoder = angleMotor.getEncoder();
    integratedEncoder.setPosition(0);

    
    // Motor inverts
    driveMotor.setInverted(driveInvert);
    angleMotor.setInverted(angleInvert);

    // add drive position and velocity conversions
    driveEncoder.setPositionConversionFactor(Constants.drivePosConversion);
    driveEncoder.setVelocityConversionFactor(Constants.driveVelConversion);

    // angle position conversions
    integratedEncoder.setPositionConversionFactor(Constants.anglePosConversion);
    // setupCANCoder();
    
    // set the drive PID's
    driveController.setP( .1);
    driveController.setI(0);
    driveController.setD(0);
    driveController.setFF(1/Constants.freeSpd); // IMPORTANT START WITH F GAIN AND SMALL P GAIN FOR TUNING DRIVETRAINS
    
    // setting angle controller PID's
    angleController.setP(.030);
    angleController.setI(0, 0);
    angleController.setD(0, 0);
    angleController.setFF(0, 0);

    driveMotor.burnFlash();
    angleMotor.burnFlash();

    resetAbsolute();

    lastAngle = getstate().angle;
  }


  public void resetAbsolute(){
    double absolutePos = getNacCoder() - offset.getDegrees();
    integratedEncoder.setPosition(absolutePos);
    // angleController.setReference(0, ControlType.kPosition);
  }

  // public void setupCANCoder() {
  //   nac.getConfigurator().apply(Robot.nacConfiguration.swerveCoderConfig);
  // }

  // set desired state of angle and drive motors
  public void setDesiredState(SwerveModuleState desiredState, boolean openLoop){
    desiredState = SwerveModuleState.optimize(desiredState, getstate().angle);

    setSpeed(desiredState, openLoop);
    setAngle(desiredState);
  }

  // set angle function
  public void setAngle(SwerveModuleState desiredstate){
    Rotation2d angle;
    double absolutespeed = Math.abs(desiredstate.speedMetersPerSecond);
    if(absolutespeed <= (Constants.maxAngularSpd*.01)){
      angle = lastAngle;

    }
    else{
      angle = desiredstate.angle;
    }
    angleController.setReference(angle.getDegrees(),ControlType.kPosition);
    lastAngle = angle;
  }


  // set speed function
  public void setSpeed(SwerveModuleState desiredstate, boolean openLoop){
    if(openLoop) {
      double Speedoutput = desiredstate.speedMetersPerSecond/Constants.maxSpeed;
      driveMotor.set(Speedoutput);
    }

    else { 
      driveController.setReference(desiredstate.speedMetersPerSecond, ControlType.kVelocity, 0);
    }
  }

  // stop function for modules
  public void angleSet() {
    angleController.setReference(90, ControlType.kPosition);
  }

  public double getAngle() {
    return integratedEncoder.getPosition();
  }

  // get CANCoder position multiplied by 360.0
  public double getNacCoder(){
    return nac.getAbsolutePosition().getValueAsDouble()*360.0;
  }

  // get drive position
  public double getDrivePosition(){
    return driveEncoder.getPosition();
  }

  // get drive velocity
  public double getDriveVelocity(){
     return driveEncoder.getVelocity();
  }

  public void setVelocity(){
    driveController.setReference(.5, ControlType.kVelocity);
  }
  

  // get state
  public SwerveModuleState getstate() {
    return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(integratedEncoder.getPosition()));
  }

  public SwerveModulePosition getPosition(){
    // return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getCanCoder()));
    return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getAngle()));
  }
  

  // https://github.com/Frc5572/FRC2022/blob/main/src/main/java/frc/robot/subsystems/Swerve.java

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Swerve Angle" + moduleNum, getAngle());
    SmartDashboard.putNumber("Swerve Velocity " + moduleNum, getDriveVelocity());
    SmartDashboard.putNumber("Drive Position " + moduleNum, getDrivePosition());
    SmartDashboard.putNumber("Nac Angle " + moduleNum, getNacCoder()-offset.getDegrees());
    // SmartDashboard.putNumber("Swerve pwr " + moduleNum, driveMotor.getAppliedOutput());
  }
}
