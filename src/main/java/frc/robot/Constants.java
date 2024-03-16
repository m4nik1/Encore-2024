// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
      // Swerve system constants
    public static final double wheelBase = Units.inchesToMeters(23);
    public static final double trackWidth = Units.inchesToMeters(18);
    public static final double driveBaseRadius = Math.hypot(wheelBase / 2.0, trackWidth / 2.0);
    public static final double wheelDia = Units.inchesToMeters(4.0);
    public static final double driveRatio = 6.75;
    public static final double angleRatio = 150.0/7.0;
    public static final double maxSpeed = 4; // meters per second
    public static final double maxAngularSpd = 5;
    public static final double freeSpd = ((6784/60) * (wheelDia* Math.PI))/driveRatio;

    // Swerve Kinematics
    public static final SwerveDriveKinematics swerveKinematics = 
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    );

    public static final double drivePosConversion = (wheelDia * Math.PI) / driveRatio;
    public static final double driveVelConversion = drivePosConversion / 60.0;
    public static final double anglePosConversion = 360 / angleRatio;
    public static final boolean CANCoderInvert = false;

    public static final Rotation2d angleOffsetMod0 = Rotation2d.fromDegrees(.9243*360);
    public static final Rotation2d angleOffsetMod1 = Rotation2d.fromDegrees(.0097*360);
    public static final Rotation2d angleOffsetMod2 = Rotation2d.fromDegrees(.7478*360);
    public static final Rotation2d angleOffsetMod3 = Rotation2d.fromDegrees(.0085*360);

    // Angle motor PID Values
    public static final double angleP = 0.01;
    public static final double angleI = 0.0;
    public static final double angleD = 0.0;
    public static final double angleFF = 0.0;

    

    // Stormbreaker Constants
    public static final double speakerShot = 15;
    public static final double ampPosition = 101;
    public static final double groundPosition = 0;
    public static final double stowPosition = 20;
    public static final double autoGroundPos = .05;

    // CAN ID's
    public static final int pigeonID = 21;
}
