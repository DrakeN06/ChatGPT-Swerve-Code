// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

// absoluteEncoderReversed will need to be added to the public SwerveModule parameters
// absoluteEncoderOffsetRad will need to be added to the public SwerveModule parameters
  

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveModule frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDriveMotorID, 
    DriveConstants.kFrontLeftTurningMotorID, 
    DriveConstants.kFrontLeftDriveEncoderReversed, 
    DriveConstants.kFrontLeftTurningEncoderReversed,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderID,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
    );
  
  private final SwerveModule frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveMotorID, 
    DriveConstants.kFrontRightTurningMotorID, 
    DriveConstants.kFrontRightDriveEncoderReversed, 
    DriveConstants.kFrontRightTurningEncoderReversed,
    DriveConstants.kFrontRightDriveAbsoluteEncoderID,
    DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
  );

  private final SwerveModule backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveMotorID, 
    DriveConstants.kBackLeftTurningMotorID, 
    DriveConstants.kBackLeftDriveEncoderReversed, 
    DriveConstants.kBackLeftTurningEncoderReversed,
    DriveConstants.kBackLeftDriveAbsoluteEncoderID,
    DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
  );

  private final SwerveModule backRight = new SwerveModule(
    DriveConstants.kBackRightDriveMotorID, 
    DriveConstants.kBackRightTurningMotorID, 
    DriveConstants.kBackRightDriveEncoderReversed, 
    DriveConstants.kBackRightTurningEncoderReversed,
    DriveConstants.kBackRightDriveAbsoluteEncoderID,
    DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kBackRightDriveAbsoluteEncoderReversed
  );

  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  public SwerveSubsystem() {

    new Thread(() -> {

      try {

        Thread.sleep(1000);
        zeroHeading();

      } catch (Exception e) {

      }
      
    }).start();

  }

  public void zeroHeading() {

    gyro.reset();

  }

  public double getHeading() {

    return Math.IEEEremainder(gyro.getAngle(), 360);

  }

  public Rotation2d getRotation2d() {

    return Rotation2d.fromDegrees(getHeading());

  }

  @Override
  public void periodic() {

  }

  public void stopModules() {

    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();

  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond); // MAY NEED TO BE CHANGED
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);

  }

}
