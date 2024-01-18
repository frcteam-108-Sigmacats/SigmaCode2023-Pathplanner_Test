// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

public class AlignRobot extends Command {
  private Vision vision;
  private SwerveSubsystem swerve;
  private double rotation;
  private Translation2d translation = new Translation2d();
  private PIDController turnPID = new PIDController(0.0001, 0.001, 0);
  private double offset = -13.67;
  /** Creates a new AlignRobot. */
  public AlignRobot(Vision vision, SwerveSubsystem swerve) {
    this.vision = vision;
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(vision.limeLight.getEntry("tv").getDouble(0) > 0){
      // rotation = turnPID.calculate(vision.tx, offset);
      // System.out.println("Rotating:" + rotation);
      // swerve.drive(translation, rotation, false);
      if(vision.tx > offset){
        rotation = turnPID.calculate(vision.tx, offset);
        swerve.drive(translation, rotation, false);
        if(vision.tx == offset){
          rotation = 0;
          swerve.drive(translation, rotation, false);
        }
      }
    }
    else if(vision.limeLight.getEntry("tv").getDouble(0) == 0 || vision.tx == -9){
      rotation = 0;
      System.out.println("Not rotating");
      swerve.drive(translation, rotation, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(rotation == 0){
      return true;
    }
    return false;
  }
}
