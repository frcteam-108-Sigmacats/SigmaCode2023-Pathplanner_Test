// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveBack extends Command {
  private SwerveSubsystem swerveSub;
  private Claw clawMech;
  private int counter;
  private Translation2d translation = new Translation2d(0, 0.4);
  /** Creates a new DriveBack. */
  public DriveBack(SwerveSubsystem swerve, Claw clawSub) {
    swerveSub = swerve;
    clawMech = clawSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSub, clawMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSub.drive(translation, 0, true);
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(counter >= 25){
      return true;
    }
    return false;
  }
}
