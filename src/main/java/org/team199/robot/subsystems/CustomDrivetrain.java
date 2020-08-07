package org.team199.robot.subsystems;

import org.team199.lib.SwerveMath;
import org.team199.robot.Constants;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Extension of BaseDrivetrain based on Team 199 math.
 */
public class CustomDrivetrain extends BaseDrivetrain {
// Odometry equivalent for non-WPILib implementation
  private Pose2d pose;
  private double gyroOffset;

  public CustomDrivetrain() {
    Rotation2d rot = new Rotation2d(getHeading());
    pose = new Pose2d(new Translation2d(), rot);
    gyroOffset = pose.getRotation().minus(rot).getRadians();
  }

  @Override
  protected void updateOdometry() {
    SwerveModuleState states[] =  new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getCurrentState();
    }
    
    // Do the inverse of swerve math; rather than calculate swerve module states from chassis speeds, calculate chassis speeds from swerve module states.
    double temp[] = new double[3];
    if (SmartDashboard.getBoolean("Field Oriented", true)) {
    temp = SwerveMath.inverseSwerve(Constants.DriveConstants.wheelBase,
                                    Constants.DriveConstants.trackWidth, 
                                    getHeading(), states);
    } else {
    temp = SwerveMath.inverseSwerve(Constants.DriveConstants.wheelBase,
                                    Constants.DriveConstants.trackWidth,
                                    states);
    }

    // Only forward and strafe are required
    double actualForward = temp[0];
    double actualStrafe = temp[1];

    // Period is 20 miliseconds since period is called every 20 miliseconds.
    double period = 20e-3;
    double deltaAngle = (getHeading() + gyroOffset) - pose.getRotation().getRadians();
    double deltaX = actualForward * period;
    double deltaY = actualStrafe * period;
    Pose2d newPose = pose.exp(new Twist2d(deltaX, deltaY, deltaAngle));
    pose = new Pose2d(newPose.getTranslation(), new Rotation2d(getHeading() + gyroOffset));
  }

  @Override
  protected void updateSmartDashboard() {
      // Display the status of the odometry.
    SmartDashboard.putNumber("Pose X", pose.getTranslation().getX());
    SmartDashboard.putNumber("Pose Y", pose.getTranslation().getY());
    SmartDashboard.putNumber("Pose Rotation (Radians)", pose.getRotation().getRadians());

    for (int i = 0; i < 4; i++) {
        modules[i].updateSmartDashboard();
    }
  }

  @Override
  public void drive(double forward, double strafe, double rotation) {
    SwerveModuleState[] moduleStates;
    if (SmartDashboard.getBoolean("Field Oriented", true)) {
        moduleStates = SwerveMath.calculateSwerve(forward, strafe, rotation, Math.toRadians(getHeading()), 
                                            Constants.DriveConstants.wheelBase, Constants.DriveConstants.trackWidth);
    } else {
        moduleStates = SwerveMath.calculateSwerve(forward, strafe, rotation, Constants.DriveConstants.wheelBase, Constants.DriveConstants.trackWidth);
    }
    SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.DriveConstants.maxSpeed);

    // Move the modules based on desired (normalized) speed, desired angle, max speed, drive modifier, and whether or not to reverse turning.
    for (int i = 0; i < 4; i++) {
      modules[i].move(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle.getRadians());
    }
  }
}