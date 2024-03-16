// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  ElmCitySwerve[] elmSwerveMods;

  Pigeon2 gyro;

  SwerveDriveOdometry odom;

  public DriveTrain() {
    elmSwerveMods = new ElmCitySwerve[] {
      new ElmCitySwerve(0, 14, 15, 24, Constants.angleOffsetMod0, false, true),
      new ElmCitySwerve(1, 13, 12, 25, Constants.angleOffsetMod1, true, true),
      new ElmCitySwerve(2, 4, 1, 26, Constants.angleOffsetMod2, false, true),
      new ElmCitySwerve(3, 3, 2, 27, Constants.angleOffsetMod3 , true, true)
    };

    gyro = new Pigeon2(Constants.pigeonID);

    odom = new SwerveDriveOdometry(Constants.swerveKinematics, getYaw(), getPositions());
    resetGyro();  


    new Thread(() -> {
      try {
        Thread.sleep(1000);
        resetPose(new Pose2d());
        odom.resetPosition(new Rotation2d(), getPositions(), new Pose2d());
      } catch(Exception e) {}
    }).start();

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetPose,
      this::getRobotSpds,
      this::driveRobotRelative,
      new HolonomicPathFollowerConfig(
        new PIDConstants(8, 0, 0), // translation pid
        new PIDConstants(17.5, 0, 0), // rotation PID
        Constants.maxSpeed,
        Constants.driveBaseRadius,
        new ReplanningConfig(true, false)
      ),
      () -> {
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void resetPose(Pose2d pose) {
    odom.resetPosition(getYaw(), getPositions(), pose);
  }

  public Pose2d getPose() {
    return odom.getPoseMeters();
  }

  public Rotation2d getYaw() {
    return  Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  public ChassisSpeeds getRobotSpds() {
    return Constants.swerveKinematics.toChassisSpeeds(
      elmSwerveMods[0].getstate(),
      elmSwerveMods[1].getstate(),
      elmSwerveMods[2].getstate(),
      elmSwerveMods[3].getstate()
    );
  }


  public void drive(Translation2d translation, double rotation) {
    SwerveModuleState[] moduleStates;

    ChassisSpeeds spds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw());
    spds = ChassisSpeeds.discretize(spds, .02);
    moduleStates = Constants.swerveKinematics.toSwerveModuleStates(spds);

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.maxSpeed);

    for(ElmCitySwerve m : elmSwerveMods) {
      m.setDesiredState(moduleStates[m.moduleNum], true);
    }
  }

  public void driveRobotRelative(ChassisSpeeds spds) {
    SwerveModuleState states[] = Constants.swerveKinematics.toSwerveModuleStates(spds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxSpeed);

    for(ElmCitySwerve m : elmSwerveMods) {
      m.setDesiredState(states[m.moduleNum], false);
    }
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for(ElmCitySwerve m : elmSwerveMods) {
      positions[m.moduleNum] = m.getPosition();
    }

    return positions;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odom.update(getYaw(), getPositions());

    SmartDashboard.putNumber("Robot Angle", getYaw().getDegrees());
  }
}
