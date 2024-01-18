// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalanceFront extends Command {
  private SwerveSubsystem swerve;
  private double ySpeed;
  private Translation2d translation;
  private double offset = 12;
  public boolean rampDone = false;
  /** Creates a new AutoBalanceFront. */
  public AutoBalanceFront(SwerveSubsystem swerveSub) {
    swerve = swerveSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rampDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(rampDone == false)
    {
      ySpeed = 0.35;

      translation = new Translation2d(ySpeed, 0);
      swerve.drive(translation, 0, true);
      
      if(swerve.getPitch().getDegrees() >= offset){
        rampDone = true;

        ySpeed = 0;
        translation = new Translation2d(ySpeed, 0);
        swerve.drive(translation, 0, true);

        System.out.println("Boolean " + rampDone);
      }
    }
    else{
      swerve.xLock();     
    }
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
