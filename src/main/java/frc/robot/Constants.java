// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final class OperatorConstants {

    public static final int kDriverControllerPort = 0;

    public static final int kDriverYAxis = 0;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 0;
    public static final int kDriverFieldOrientedButtonIdx = 0;

    public static final double kDeadBand = 0;

  }

  public static final class ModuleConstants {

    public static final double kWheelDiameterMeters = Units.inchesToMeters(0);
    public static final double kDriveMotorGearRatio = 0;
    public static final double kTurningMotorGearRatio = 0;
    
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    public static final double kPTurning = 0;
    
  }

  public static final class DriveConstants {
    
    public static final double kTrackWidth = Units.inchesToMeters(0);
    public static final double kWheelBase = Units.inchesToMeters(0);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2), 
      new Translation2d(kWheelBase / 2, kTrackWidth / 2), 
      new Translation2d(kWheelBase / 2, kTrackWidth / 2), 
      new Translation2d(kWheelBase / 2, kTrackWidth / 2));
    
    public static final double kPhysicalMaxSpeedMetersPerSecond = 0;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 0 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 1;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 1;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1;

    // All constants relating to the front left drive motor & absolute encoder
    public static final int kFrontLeftDriveMotorID = 0;
    public static final int kFrontLeftTurningMotorID = 0;
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final int kFrontLeftDriveAbsoluteEncoderID = 0;
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;

    // All constants relating to the front right drive motor & absolute encoder
    public static final int kFrontRightDriveMotorID = 0;
    public static final int kFrontRightTurningMotorID = 0;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final int kFrontRightDriveAbsoluteEncoderID = 0;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;

    // All constants relating to the back left drive motor & absolute encoder
    public static final int kBackLeftDriveMotorID = 0;
    public static final int kBackLeftTurningMotorID = 0;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final int kBackLeftDriveAbsoluteEncoderID = 0;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;

    // All constants relating to the back right drive motor & absolute encoder
    public static final int kBackRightDriveMotorID = 0;
    public static final int kBackRightTurningMotorID = 0;
    public static final boolean kBackRightDriveEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;
    public static final int kBackRightDriveAbsoluteEncoderID = 0;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

  }

}
