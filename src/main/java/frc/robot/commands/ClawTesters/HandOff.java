// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClawTesters;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw;

public class HandOff extends Command {
  private Claw claw = RobotContainer.m_Claw;
  private double speed;
  private int counter;
  private boolean isReturn;
  /** Creates a new HandOff. */
  public HandOff(double speed, boolean isReturn) {
    this.speed = speed;
    this.isReturn = isReturn;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isReturn == false){
      counter++;
      claw.handOffPosition(speed, counter);
    }
    else if(isReturn == true){
      counter++;
      claw.stopHandOff(speed, counter);
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
