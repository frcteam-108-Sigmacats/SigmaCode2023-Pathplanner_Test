// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class SetClawStates extends Command {
  private int clawState;
  private Claw clawMech;
  private DigitalInput magnetSensor;
  private int counter;
  /** Creates a new SetClawStates. */
  public SetClawStates(Claw clawSub, int clawState) {
    this.clawState = clawState;
    clawMech = clawSub;
    magnetSensor = Claw.cylinderSensor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(clawMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(clawState == 1){
      if(magnetSensor.get() == true){
        counter++;
        clawMech.setClawStates(clawState, counter);
      }
    }
    else{
      clawMech.setClawStates(clawState, counter);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawMech.clawExtenders.set(Value.kReverse);
    //clawMech.setClawStates(clawState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
