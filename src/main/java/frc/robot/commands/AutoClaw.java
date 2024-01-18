// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Robot;
// import frc.robot.subsystems.Claw;
// import frc.robot.subsystems.SwerveSubsystem;

// public class AutoClaw extends Command {
//   private Claw claw;
//   private SwerveSubsystem swerve;
//   private int counter;
//   /** Creates a new AutoClaw. */
//   public AutoClaw(Claw claw, SwerveSubsystem swerve) {
//     this.claw = claw;
//     this.swerve = swerve;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(claw, swerve);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     counter = 0;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     counter++;
//     if(Robot.hub.getPressure(0) >= 90){
//       if(swerve.canArmExtend() && claw.getGamePiece() == true){
//         //claw.setClawStates(claw.armStates);
//       }
//       else if(swerve.canArmExtend() && claw.getGamePiece() == false){
//         if(counter > 300){
//           //claw.setClawStates(1);
//         }
//       }
//       else if(swerve.canArmExtend() == false && claw.getGamePiece() == true){
//         if(counter > 300){
//           //claw.setClawStates(1);
//         }
//       }
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
