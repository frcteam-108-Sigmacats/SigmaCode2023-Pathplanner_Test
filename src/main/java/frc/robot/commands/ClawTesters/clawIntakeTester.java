// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClawTesters;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class clawIntakeTester extends Command {
  private Claw claw;
  private double speed;
  /** Creates a new clawIntakeTester. */
  public clawIntakeTester(Claw claw, double speed) {
    this.claw = claw;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    claw.intakeTester(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.intakeTester(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
