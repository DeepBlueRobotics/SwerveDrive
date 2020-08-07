/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot.subsystems;

import org.team199.robot.Constants;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Extension of BaseDrivetrain based on WPILib math.
 */
public class Drivetrain extends BaseDrivetrain {
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;

  /**
   * Constructs a Drivetrain object
   */
  public Drivetrain() {
    // Define the corners of the robot relative to the center of the robot using Translation2d objects.
    // Positive x-values represent moving toward the front of the robot whereas positive y-values represent moving toward the left of the robot.
    Translation2d locationFL = new Translation2d(Constants.DriveConstants.wheelBase / 2, Constants.DriveConstants.trackWidth / 2);
    Translation2d locationFR = new Translation2d(Constants.DriveConstants.wheelBase / 2, -Constants.DriveConstants.trackWidth / 2);
    Translation2d locationBL = new Translation2d(-Constants.DriveConstants.wheelBase / 2, Constants.DriveConstants.trackWidth / 2);
    Translation2d locationBR = new Translation2d(-Constants.DriveConstants.wheelBase / 2, -Constants.DriveConstants.trackWidth / 2);

    kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(getHeading()));
  }

  protected void updateOdometry() {
    SwerveModuleState states[] =  new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getCurrentState();
    }
    odometry.update(Rotation2d.fromDegrees(getHeading()), states);
  }

  protected void updateSmartDashboard() {
    // Display the status of the odometry.
    SmartDashboard.putNumber("Pose X", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Pose Y", odometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("Pose Rotation (Radians)", odometry.getPoseMeters().getRotation().getRadians());

    for (int i = 0; i < 4; i++) {
      modules[i].updateSmartDashboard();
    }
   }

  public void drive(double forward, double strafe, double rotation) {
    SwerveModuleState[] moduleStates = getSwerveStates(forward, strafe, rotation);
    SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.DriveConstants.maxSpeed);

    // Move the modules based on desired (normalized) speed, desired angle, max speed, drive modifier, and whether or not to reverse turning.
    for (int i = 0; i < 4; i++) {
      modules[i].move(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle.getRadians());
    }
  }

  /**
   * Constructs and returns a ChassisSpeeds objects using forward, strafe, and rotation values.
   * 
   * @param forward     The desired forward speed, in m/s.
   * @param strafe      The desired strafe speed, in m/s.
   * @param rotation    The desired rotation speed, in rad/s.
   * @return A ChassisSpeeds object.
  */
  private ChassisSpeeds getChassisSpeeds(double forward, double strafe, double rotation) {
    ChassisSpeeds speeds;
    if (SmartDashboard.getBoolean("Field Oriented", true)) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Rotation2d.fromDegrees(getHeading()));
    } else {
      speeds = new ChassisSpeeds(forward, strafe, rotation);
    }
    return speeds;
  }

  /**
   * Constructs and returns four SwerveModuleState objects, one for each side, using forward, strafe, and rotation values.
   * 
   * @param forward     The desired forward speed, in m/s.
   * @param strafe      The desired strafe speed, in m/s.
   * @param rotation    The desired rotation speed, in rad/s.
   * @return A SwerveModuleState array, one for each side of the drivetrain (FL, FR, etc.).
  */
  private SwerveModuleState[] getSwerveStates(double forward, double strafe, double rotation) {
    return kinematics.toSwerveModuleStates(getChassisSpeeds(forward, -strafe, -rotation));
  }
}
