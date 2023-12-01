// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveSubsystem extends SubsystemBase {
  public static final double kMaxSpeed = 5.0; // 3 meters per second
  public static final double kMaxAngularSpeed = 4*Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, 0.381);//0.762
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, -0.381);//9.5
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, 0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(5, 6, 3, 225);
  private final SwerveModule m_frontRight = new SwerveModule(2, 1, 1, 292.32);
  private final SwerveModule m_backLeft = new SwerveModule(7, 8, 4, 199.86);
  private final SwerveModule m_backRight = new SwerveModule(4, 3, 2, 328.8);

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(9999);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(9999);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(9999);

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_gyro.calibrate();
    m_gyro.reset();
  }
  public void drive(
    double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    double x = xSpeed;
    double y = ySpeed;
    xSpeed =
    m_xspeedLimiter.calculate(MathUtil.applyDeadband(xSpeed, 0.02))
        * kMaxSpeed;
    ySpeed =
    -m_yspeedLimiter.calculate(MathUtil.applyDeadband(ySpeed, 0.02))
        * kMaxSpeed;

    rot =
    -m_rotLimiter.calculate(MathUtil.applyDeadband(rot, 0.02))
        * kMaxAngularSpeed;

    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_gyro.getYaw()))
                    : new ChassisSpeeds(xSpeed, ySpeed, rot)
                );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }
  public AHRS getGyro(){
    return m_gyro;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run\
    SmartDashboard.putNumber("Drive1 Velocity", m_frontRight.getDriveMotor().get());
    SmartDashboard.putNumber("Drive2 Velocity", m_backRight.getDriveEncoder().getVelocity());
    SmartDashboard.putNumber("Drive3 Velocity", m_frontLeft.getDriveEncoder().getVelocity());
    SmartDashboard.putNumber("Drive4 Velocity", m_backLeft.getDriveEncoder().getVelocity());
    SmartDashboard.putNumber("Drive Pos 1", m_frontRight.getTurnEncoder().getPosition());
    SmartDashboard.putNumber("Drive Pos 2", m_backRight.getTurnEncoder().getPosition());
    SmartDashboard.putNumber("Drive Pos 3", m_frontLeft.getTurnEncoder().getPosition());
    SmartDashboard.putNumber("Drive Pos 4", m_backLeft.getTurnEncoder().getPosition());
    SmartDashboard.putNumber("Angle", m_gyro.getAngle());
    SmartDashboard.putNumber("Yaw", m_gyro.getYaw());
  }
}
