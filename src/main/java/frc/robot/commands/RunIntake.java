// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class RunIntake extends Command {
  private double speed;
  private int intakeState;
  private Claw claw = RobotContainer.m_Claw;
  private int counter;
  /** Creates a new RunIntake. */
  public RunIntake(int intakeState, double speed) {
    this.speed = speed;
    this.intakeState = intakeState;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    RobotContainer.m_Claw.intakeStates(intakeState, speed, counter);
    System.out.println("Counter is: " + counter);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.clawExtenders.set(Value.kReverse);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(Claw.isFinished == true){
    //   return true;
    // }
    return false;
  }
}
