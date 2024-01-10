// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import java.lang.Math;
import edu.wpi.first.math.controller.PIDController;
public class GotoAprilTag extends CommandBase {
  /** Creates a new GotoAprilTag. */

  private DriveSubsystem driveSubsystem;
  private final double heightDifference = 1;

  private double targetXPose;
  private double targetYPose;

  private PIDController positionPIDControllerX = new PIDController(0,0,0);
  private PIDController positionPIDControllerY = new PIDController(0,0,0);

  public GotoAprilTag(DriveSubsystem s1) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s1);
    driveSubsystem=s1;
  }

  // Called when the command is initially scheduled.
  private double normalizeAngle(double angle){
    return angle;
  }
  @Override
  public void initialize() {
    double angleRelativeToField = normalizeAngle(driveSubsystem.getGyro().getYaw()+LimelightSubsystem.getX());
    double distanceToTarget = heightDifference/Math.tan(LimelightSubsystem.getY()*180/Math.PI);
    double distanceToMoveX;
    double distanceToMoveY;
    if(Math.abs(angleRelativeToField)<90){
      distanceToMoveX=-distanceToTarget*Math.sin(Math.abs(angleRelativeToField)*Math.PI/180);
      distanceToMoveY=distanceToTarget*Math.cos(Math.abs(angleRelativeToField)*Math.PI/180);
      if(angleRelativeToField<0){
        distanceToMoveX*=-1;
      }
    }
    else if(Math.abs(angleRelativeToField)>90){
      distanceToMoveX=-distanceToTarget*Math.cos((Math.abs(angleRelativeToField)-90)*Math.PI/180);
      distanceToMoveY=-distanceToTarget*Math.sin((Math.abs(angleRelativeToField)-90)*Math.PI/180);
      if(angleRelativeToField<0){
        distanceToMoveX*=-1;
      }
    }
    else{
      distanceToMoveX=distanceToTarget;
      distanceToMoveY=0;
      if(angleRelativeToField<0){
        distanceToMoveX*=-1;
      }
    }
    targetXPose=driveSubsystem.getPose2d().getX()+distanceToMoveX;
    targetYPose=driveSubsystem.getPose2d().getY()+distanceToMoveY;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(driveSubsystem.getPose2d().getX()-targetXPose)<0.1 && Math.abs(driveSubsystem.getPose2d().getY()-targetYPose)<0.1){
      this.cancel();
    }
    else{
      driveSubsystem.drive(
        positionPIDControllerX.calculate(driveSubsystem.getPose2d().getX(),targetXPose),
        positionPIDControllerY.calculate(driveSubsystem.getPose2d().getY(),targetYPose),
        driveSubsystem.getHeadingController().calculate(LimelightSubsystem.getX(),0),
        true,
        0
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
