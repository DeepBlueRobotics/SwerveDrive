/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot.subsystems;

import org.team199.lib.MotorControllerFactory;
import org.team199.lib.SwerveMath;
import org.team199.robot.Constants;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Drivetrain subsystem. 
 * Contains the majority of methods associated with driving the robot or updating the position of the robot.
 */
public class Drivetrain extends SubsystemBase {
  /** Enum for representing each of the four corners of the drivetrain:
   * 
   * <p><ul>
   * <li> FL - forward-left (or front-left).
   * <li> FR - forward-right (or front-right).
   * <li> BL - back-left (or back-left).
   * <li> BR - back-right (or back-right).
   * </ul><p>
  */
  public enum Side {
    FL, FR, BL, BR;
  }

  /** Controls which codebase to upon deploy:
   * 
   * <p><ul>
   * <li> Custom - code written by Team 199 from the ground-up (mostly).
   * <li> WPILib - WPILib API that handles the complicated math.
   * </ul><p>
  */
  public enum SwerveImplementation {
    Custom, WPILib;
  }

  private final SwerveImplementation implementation;
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;

  private final AHRS gyro = new AHRS(SerialPort.Port.kUSB1);
  private final boolean isGyroReversed = true;

  // Two sets of motor controllers, four for each side. 
  // Drive motor controllers move the robot forward/backward, while turn motor control the angle of the module.
  
  public WPI_TalonSRX driveFL = MotorControllerFactory.createTalon(Constants.Ports.kDriveFrontLeft);
  public WPI_TalonSRX driveFR = MotorControllerFactory.createTalon(Constants.Ports.kDriveFrontRight);
  public WPI_TalonSRX driveBL = MotorControllerFactory.createTalon(Constants.Ports.kDriveBackLeft);
  public WPI_TalonSRX driveBR = MotorControllerFactory.createTalon(Constants.Ports.kDriveBackRight);

  public WPI_TalonSRX turnFL = MotorControllerFactory.createTalon(Constants.Ports.kTurnFrontLeft);
  public WPI_TalonSRX turnFR = MotorControllerFactory.createTalon(Constants.Ports.kTurnFrontRight);
  public WPI_TalonSRX turnBL = MotorControllerFactory.createTalon(Constants.Ports.kTurnBackLeft);
  public WPI_TalonSRX turnBR = MotorControllerFactory.createTalon(Constants.Ports.kTurnBackRight);

  private double speeds[] = {0.0, 0.0, 0.0, 0.0};

  // Odometry equivalent for non-WPILib implementation
  public Pose2d pose;
  private double gyroOffset;

  /**
   * Constructs a Drivetrain object
   * @param implementation    The chosen implementation type. 
   * NOTE: This cannot be changed during runtime and is updated when code is deployed to the robot.
   */
  public Drivetrain(SwerveImplementation implementation) {
    this.implementation = implementation;
    gyro.reset();
    double heading = Math.toRadians(getHeading());

    // Configure PID control constants for drive motor controllers
    driveFL.config_kP(0, Constants.DriveConstants.driveKP[0]);
    driveFR.config_kP(0, Constants.DriveConstants.driveKP[1]);
    driveBL.config_kP(0, Constants.DriveConstants.driveKP[2]);
    driveBR.config_kP(0, Constants.DriveConstants.driveKP[3]);
    driveFL.config_kI(0, Constants.DriveConstants.driveKI[0]);
    driveFR.config_kI(0, Constants.DriveConstants.driveKI[1]);
    driveBL.config_kI(0, Constants.DriveConstants.driveKI[2]);
    driveBR.config_kI(0, Constants.DriveConstants.driveKI[3]);
    driveFL.config_kD(0, Constants.DriveConstants.driveKD[0]);
    driveFR.config_kD(0, Constants.DriveConstants.driveKD[1]);
    driveBL.config_kD(0, Constants.DriveConstants.driveKD[2]);
    driveBR.config_kD(0, Constants.DriveConstants.driveKD[3]);

    // Configure PID control constants for turn motor controllers
    turnFL.config_kP(0, Constants.DriveConstants.turnKP[0]);
    turnFR.config_kP(0, Constants.DriveConstants.turnKP[1]);
    turnBL.config_kP(0, Constants.DriveConstants.turnKP[2]);
    turnBR.config_kP(0, Constants.DriveConstants.turnKP[3]);
    turnFL.config_kI(0, Constants.DriveConstants.turnKI[0]);
    turnFR.config_kI(0, Constants.DriveConstants.turnKI[1]);
    turnBL.config_kI(0, Constants.DriveConstants.turnKI[2]);
    turnBR.config_kI(0, Constants.DriveConstants.turnKI[3]);
    turnFL.config_kD(0, Constants.DriveConstants.turnKD[0]);
    turnFR.config_kD(0, Constants.DriveConstants.turnKD[1]);
    turnBL.config_kD(0, Constants.DriveConstants.turnKD[2]);
    turnBR.config_kD(0, Constants.DriveConstants.turnKD[3]);

    // Decide whether or not to use custom implementation of swerve drive or use WPILib implementation using Odometry/Kinematics.
    if (implementation == SwerveImplementation.WPILib) {
      // Define the corners of the robot relative to the center of the robot using Translation2d objects.
      Translation2d locationFL = new Translation2d(-Constants.DriveConstants.trackWidth / 2, Constants.DriveConstants.wheelBase / 2);
      Translation2d locationFR = new Translation2d(Constants.DriveConstants.trackWidth / 2, Constants.DriveConstants.wheelBase / 2);
      Translation2d locationBL = new Translation2d(-Constants.DriveConstants.trackWidth, -Constants.DriveConstants.wheelBase / 2);
      Translation2d locationBR = new Translation2d(Constants.DriveConstants.trackWidth / 2, -Constants.DriveConstants.wheelBase / 2);

      kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
      odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(heading));
    } else {
      Rotation2d rot = new Rotation2d(heading);
      pose = new Pose2d(new Translation2d(), rot);
      gyroOffset = pose.getRotation().minus(rot).getRadians();
    }
  }

  /**
   * Updates the odometry of the robot, either via SwerveDriveOdometry.update() or via the custom implementation.
   * See section 10 of <a href = "https://file.tavsys.net/control/controls-engineering-in-frc.pdf"> Controls Engineering in the
FIRST Robotics Competition </a> by Tyler Veness for more information.
   */
  public void updateOdometry() {
    // There are no encoders on the drive motor controllers so assume current speed = desired speed
    SwerveModuleState flState = new SwerveModuleState(speeds[0], new Rotation2d(getCurrentModuleAngle(Side.FL)));
    SwerveModuleState frState = new SwerveModuleState(speeds[1], new Rotation2d(getCurrentModuleAngle(Side.FR)));
    SwerveModuleState blState = new SwerveModuleState(speeds[2], new Rotation2d(getCurrentModuleAngle(Side.BL)));
    SwerveModuleState brState = new SwerveModuleState(speeds[3], new Rotation2d(getCurrentModuleAngle(Side.BR)));

    if (implementation == SwerveImplementation.WPILib) {
      odometry.update(Rotation2d.fromDegrees(getHeading()), flState, frState, blState, brState);
    } else {
      // Do the inverse of the swerve math; rather than calculate swerve module states from chassis speeds, calculate chassis speeds from swerve module states.
      double temp[]= new double[3];
      if (SmartDashboard.getBoolean("Field Oriented", true)) {
        temp = SwerveMath.inverseSwerve(Constants.DriveConstants.wheelBase,
                                        Constants.DriveConstants.trackWidth, 
                                        getHeading(), 
                                        flState, frState, blState, brState);
      } else {
        temp = SwerveMath.inverseSwerve(Constants.DriveConstants.wheelBase,
                                        Constants.DriveConstants.trackWidth,
                                        flState, frState, blState, brState);
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
  }

  /**
   * Updates SmartDashboard with information about the drivetrain pertinent to the operator.
  */
  public void updateSmartDashboard() {
    // Display the status of the odometry.
    SmartDashboard.putNumber("Pose X", (implementation == SwerveImplementation.WPILib) ? 
      odometry.getPoseMeters().getTranslation().getX() : pose.getTranslation().getX());
    SmartDashboard.putNumber("Pose Y", (implementation == SwerveImplementation.WPILib) ? 
      odometry.getPoseMeters().getTranslation().getY() : pose.getTranslation().getY());
    SmartDashboard.putNumber("Pose Rotation (Radians)", (implementation == SwerveImplementation.WPILib) ? 
      odometry.getPoseMeters().getRotation().getRadians() : pose.getRotation().getRadians());
    
    // Display the position of the quadrature encoder.
    SmartDashboard.putNumber("FL Quadrature Position", getQuadraturePosition(Side.FL));
    SmartDashboard.putNumber("FR Quadrature Position", getQuadraturePosition(Side.FR));
    SmartDashboard.putNumber("BL Quadrature Position", getQuadraturePosition(Side.BL));
    SmartDashboard.putNumber("BR Quadrature Position", getQuadraturePosition(Side.BR));

    // Display the position of the analog encoder.
    SmartDashboard.putNumber("FL Analog Position", getAnalogPosition(Side.FL));
    SmartDashboard.putNumber("FR Analog Position", getAnalogPosition(Side.FR));
    SmartDashboard.putNumber("BL Analog Position", getAnalogPosition(Side.BL));
    SmartDashboard.putNumber("BR Analog Position", getAnalogPosition(Side.BR));

    // Display the module angle as calculated using the absolute encoder.
    SmartDashboard.putNumber("FL Module Angle", getCurrentModuleAngle(Side.FL));
    SmartDashboard.putNumber("FR Module Angle", getCurrentModuleAngle(Side.FR));
    SmartDashboard.putNumber("BL Module Angle", getCurrentModuleAngle(Side.BL));
    SmartDashboard.putNumber("BR Module Angle", getCurrentModuleAngle(Side.BR));
  }

  @Override
  /**
   * Periodic method of the Drivetrain. Called every 20 miliseconds.
   */
  public void periodic() {
    updateOdometry();
    updateSmartDashboard();
  }

  /** 
   * Returns robot heading from the gyro in degrees.
  */
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (isGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Drives the robot.
   * @param forward   Desired forward speed (in m/s).
   * @param strafe    Desired strafe speed (in m/s).
   * @param rotation  Desired rotation speed (in rad/s).
   */
  public void drive(double forward, double strafe, double rotation) {
    SwerveModuleState[] moduleStates;
    if (implementation == SwerveImplementation.WPILib) {
      moduleStates = getSwerveStates(forward, strafe, rotation);
    } else {
      if (SmartDashboard.getBoolean("Field Oriented", true)) {
        moduleStates = SwerveMath.calculateSwerve(forward, strafe, rotation, Math.toRadians(getHeading()), 
                                                Constants.DriveConstants.wheelBase, Constants.DriveConstants.trackWidth);
      } else {
        moduleStates = SwerveMath.calculateSwerve(forward, strafe, rotation,
                                                Constants.DriveConstants.wheelBase, Constants.DriveConstants.trackWidth);
      }
    }

    double[] driveSpeeds = {moduleStates[0].speedMetersPerSecond, moduleStates[1].speedMetersPerSecond,
                            moduleStates[2].speedMetersPerSecond, moduleStates[3].speedMetersPerSecond};
    speeds = driveSpeeds;
    // Drive speeds must be normalized so that they may be passed to motor controllers which require a domain of -1.0 to 1.0 for percentage control.
    driveSpeeds = SwerveMath.normalize(driveSpeeds);

    /**************************************************
      Compute setpoints for each side and use PID.
    ***************************************************/

    double setpoints[] = SwerveMath.computeSetpoints(driveSpeeds[0], 
                                                     moduleStates[0].angle.getRadians() / (2 * Math.PI),
                                                     this.getQuadraturePosition(Side.FL),
                                                     Constants.DriveConstants.FL_GEAR_RATIO);
    driveFL.set(ControlMode.PercentOutput, setpoints[0] * Constants.DriveConstants.kDriveModifier);
    turnFL.set(ControlMode.Position, (Constants.DriveConstants.reversedFL ? -1 : 1) * setpoints[1] * Constants.DriveConstants.FL_GEAR_RATIO);

    setpoints = SwerveMath.computeSetpoints(driveSpeeds[1], 
                                            moduleStates[1].angle.getRadians() / (2 * Math.PI),
                                            this.getQuadraturePosition(Side.FR),
                                            Constants.DriveConstants.FR_GEAR_RATIO);
    driveFR.set(ControlMode.PercentOutput, setpoints[0] * Constants.DriveConstants.kDriveModifier);
    turnFR.set(ControlMode.Position, (Constants.DriveConstants.reversedFR ? -1 : 1) * setpoints[1] * Constants.DriveConstants.FR_GEAR_RATIO);

    setpoints = SwerveMath.computeSetpoints(driveSpeeds[2], 
                                            moduleStates[2].angle.getRadians() / (2 * Math.PI),
                                            this.getQuadraturePosition(Side.BL),
                                            Constants.DriveConstants.BL_GEAR_RATIO);
    driveBL.set(ControlMode.PercentOutput, setpoints[2] * Constants.DriveConstants.kDriveModifier);
    turnBL.set(ControlMode.Position, (Constants.DriveConstants.reversedBL ? -1 : 1) * setpoints[1] * Constants.DriveConstants.BL_GEAR_RATIO);

    setpoints = SwerveMath.computeSetpoints(driveSpeeds[3], 
                                            moduleStates[3].angle.getRadians() / (2 * Math.PI),
                                            this.getQuadraturePosition(Side.BR),
                                            Constants.DriveConstants.BR_GEAR_RATIO);
    driveBR.set(ControlMode.PercentOutput, setpoints[0] * Constants.DriveConstants.kDriveModifier);
    turnBR.set(ControlMode.Position, (Constants.DriveConstants.reversedBR ? -1 : 1) * setpoints[1] * Constants.DriveConstants.BR_GEAR_RATIO);

  }

  /** 
   * Returns the position of the quadrature encoder for a turn motor controller.
   * @param side  The desired side of the drivetrain from which to get the quadrature encoder position.
   * @return The position of the quadrature encoder.
  */
  public int getQuadraturePosition(Side side) {
    switch (side) {
      case FL:
        return turnFL.getSensorCollection().getQuadraturePosition();
      case FR:
        return turnFR.getSensorCollection().getQuadraturePosition();
      case BL:
        return turnBL.getSensorCollection().getQuadraturePosition();
      case BR:
        return turnBR.getSensorCollection().getQuadraturePosition();
      default:
        System.err.println("Error! Received value for side that does not equal an acceptable value.");
        return 0;
    }
  }

   /** 
   * Returns the velocity of the quadrature encoder for a turn motor controller.
   * @param side  The desired side of the drivetrain from which to get the quadrature encoder velocity.
   * @return The velocity of the quadrature encoder.
  */
   public int getQuadratureVelocity(Side side) {
    switch (side) {
      case FL:
        return turnFL.getSensorCollection().getQuadratureVelocity();
      case FR:
        return turnFR.getSensorCollection().getQuadratureVelocity();
      case BL:
        return turnBL.getSensorCollection().getQuadratureVelocity();
      case BR:
        return turnBR.getSensorCollection().getQuadratureVelocity();
      default:
      System.err.println("Error! Received value for side that does not equal an acceptable value.");
        return 0;
    }
  }

  /** 
   * Returns the position of the analog encoder for a turn motor controller.
   * @param side  The desired side of the drivetrain from which to get the analog encoder position.
   * @return The position of the analog encoder.
  */
  public int getAnalogPosition(Side side) {
    switch (side) {
      case FL:
        return turnFL.getSensorCollection().getAnalogIn();
      case FR:
        return turnFR.getSensorCollection().getAnalogIn();
      case BL:
        return turnBL.getSensorCollection().getAnalogIn();
      case BR:
        return turnBR.getSensorCollection().getAnalogIn();
      default:
        System.err.println("Error! Received value for side that does not equal an acceptable value.");
        return 0;
    }
  }

  /** 
   * Returns the angle of the analog encoder for a turn motor controller.
   * @param side  The desired side of the drivetrain from which to get the angle of sthe swerve module.
   * @return The angle, in radians, of the swerve module.
  */
  public double getCurrentModuleAngle(Side side) {
    // One full revolution is 1024 (more accurately MAX_ANALOG) on the analog encoder, so a quarter revolution is 256.
    // Therefore zero degrees is located at (TURN_ZERO - 256) mod 1024 since TURN_ZERO points to straight forward (90 degrees)
    // 1024 analog = 2 * pi radians so analog to radian is (2 * pi / 1024)((Current analog) - (TURN_ZERO - 256) mod 1024)

    int reference;
    double angle;
    switch (side) {
      case FL:
        reference = (int) (Constants.DriveConstants.FL_TURN_ZERO - 256) % Constants.DriveConstants.FL_MAX_ANALOG;
        angle = ((Math.PI * 2) / Constants.DriveConstants.FL_MAX_ANALOG) * (turnFL.getSensorCollection().getAnalogIn() - reference);
      case FR:
        reference = (int) (Constants.DriveConstants.FR_TURN_ZERO - 256) % Constants.DriveConstants.FR_MAX_ANALOG;
        angle = ((Math.PI * 2) / Constants.DriveConstants.FR_MAX_ANALOG) * (turnFR.getSensorCollection().getAnalogIn() - reference);
      case BL:
        reference = (int) (Constants.DriveConstants.BL_TURN_ZERO - 256) % Constants.DriveConstants.BL_MAX_ANALOG;
        angle = ((Math.PI * 2) / Constants.DriveConstants.BL_MAX_ANALOG) * (turnBL.getSensorCollection().getAnalogIn() - reference);
      case BR:
        reference = (int) (Constants.DriveConstants.BR_TURN_ZERO - 256) % Constants.DriveConstants.BR_MAX_ANALOG;
        angle = ((Math.PI * 2) / Constants.DriveConstants.BR_MAX_ANALOG) * (turnBR.getSensorCollection().getAnalogIn() - reference);
      default:
        System.err.println("Error! Received value for side that does not equal an acceptable value.");
        angle = 0;
    }
    
    if (angle < 0) return 2 * Math.PI + angle;
    else return angle;
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
   * @return A ChassisSpeeds object.
  */
  private SwerveModuleState[] getSwerveStates(double forward, double strafe, double rotation) {
    return kinematics.toSwerveModuleStates(getChassisSpeeds(forward, strafe, rotation));
  }
}
