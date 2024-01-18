// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalance extends Command {
  private SwerveSubsystem swerveMech;
  private Pose2d blueMinPos = new Pose2d(3.40, 0, null);
  private Pose2d blueMaxPos = new Pose2d(4.33, 0, null);
  private Pose2d redMinPos = new Pose2d(13.13, 0, null);
  private Pose2d redMaxPos = new Pose2d(12.18, 0, null);
  private Pose2d minPos = new Pose2d();
  private Pose2d maxPos = new Pose2d();
  private Translation2d translation;
  public double ySpeed;
  private double offset = -12;
  public boolean rampDone = false;
  public AutoBalance(SwerveSubsystem swerveSub) {
    swerveMech = swerveSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveMech);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rampDone = false;
    if(DriverStation.getRawAllianceStation() == AllianceStationID.Blue1 || 
    DriverStation.getRawAllianceStation() == AllianceStationID.Blue2 || 
    DriverStation.getRawAllianceStation() == AllianceStationID.Blue3){
      minPos = blueMinPos;
      maxPos = blueMaxPos;
    }
    else if(DriverStation.getRawAllianceStation() == AllianceStationID.Red1 
    || DriverStation.getRawAllianceStation() == AllianceStationID.Red2 || 
    DriverStation.getRawAllianceStation() == AllianceStationID.Red3){
      minPos = redMinPos;
      maxPos = redMaxPos;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * if(gyro >offset)
     * 
     * go at 30%
     *  if gyro less than 0 go back
     *  if gyro greater than
     */
    System.out.println("Degrees " + swerveMech.getPitch().getDegrees() + "offset " + rampDone);
    if(rampDone == false)
    {
      ySpeed = -0.35;

      translation = new Translation2d(ySpeed, 0);
      swerveMech.drive(translation, 0, true);
      
      if(swerveMech.getPitch().getDegrees() >= offset){
        rampDone = true;

        ySpeed = 0;
        translation = new Translation2d(ySpeed, 0);
        swerveMech.drive(translation, 0, true);

        System.out.println("Boolean " + rampDone);
      }
    }
    else{
      swerveMech.xLock();     
    }

/* 
    
    if(Math.abs(swerveMech.getPitch().getDegrees()) >= offset){

      //System.out.println("Radians " + swerveMech.getPitch().getRadians());
        if(swerveMech.getPitch().getDegrees() > 0 ){
          ySpeed = .3;
        }
        else {
          ySpeed = -.3;
        }
    }
    */
    // if(Math.abs(swerveMech.getPitch().getRadians()) >= Math.abs(offset) /*&& (swerveMech.getPose().getX() > minPos.getX() && swerveMech.getPose().getX() < maxPos.getX())*/){
    //   ySpeed = Math.sin(swerveMech.getPitch().getRadians());
    // }
    // else if(swerveMech.getPitch().getRadians() <= offset && swerveMech.getPitch().getRadians() >= -1.1){
    //   ySpeed = 0;
    // }
    // translation = new Translation2d(ySpeed, 0);
    // swerveMech.drive(translation, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
