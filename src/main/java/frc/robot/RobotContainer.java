// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AlignRobot;
import frc.robot.commands.AlignRobotGyro;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoBalanceFront;
import frc.robot.commands.BottomIntake;
import frc.robot.commands.BottomOuttake;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetClawStates;
import frc.robot.commands.StopIntakes;
import frc.robot.commands.Strafe;
import frc.robot.commands.SwerveDriveTeleop;
import frc.robot.commands.ZeroModules;
import frc.robot.commands.ClawTesters.HandOff;
import frc.robot.commands.ClawTesters.HoldArmTester;
import frc.robot.commands.ClawTesters.MoveArm;
import frc.robot.commands.ClawTesters.clawArmtester;
import frc.robot.commands.ClawTesters.clawIntakeHoldTester;
import frc.robot.commands.ClawTesters.clawIntakeTester;
import frc.robot.commands.ClawTesters.testingArmExtenders;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;

import javax.sound.midi.Sequencer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Vision visionSub = new Vision();
  private final SendableChooser<Command> chooser = new SendableChooser<>();
  public final static Claw m_Claw = new Claw();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);


  public Trigger dRightTrigger, dLeftTrigger, dRightBumper, dLeftBumper, dKA, dKB, dKY, dKX, dUpPov, dDownPov, dLeftPov, dRightPov;
  public Trigger oRightTrigger, oLeftTrigger, oRightBumper, oLeftBumper, oKA, oKB, oKY, oKX, oUpPov, oDownPov, oLeftPov, oRightPov;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    swerveSubsystem.setDefaultCommand(new SwerveDriveTeleop(swerveSubsystem, driver, fieldRelative));
    //m_Claw.setDefaultCommand(new SetClawStates(m_Claw, 1));
    // Configure the trigger bindings
    configureBindings();
    makeAuto();

    //Driver's buttons
    dDownPov.whileTrue(new StopIntakes());
    dUpPov.whileTrue(new ZeroModules(swerveSubsystem));
    dLeftPov.whileTrue(new Strafe(swerveSubsystem, 0.3));
    dRightPov.whileTrue(new Strafe(swerveSubsystem, -0.3));
    dLeftTrigger.whileTrue(new RunIntake(1, 0.75));//negative is cone
    dLeftTrigger.whileFalse(new SetClawStates(m_Claw, 1));
    //dLeftTrigger.whileTrue(new BottomIntake(m_Claw, 0.5, true));
    //dLeftTrigger.whileFalse(new BottomIntake(m_Claw, 0, false));
    dRightTrigger.whileTrue(new RunIntake(2, -0.65));//positive is cube intake
    dRightTrigger.whileFalse(new SetClawStates(m_Claw, 5));//Sensor needs to be fixed in order to change the state to 2
    dLeftBumper.whileTrue(new RunIntake(3, 0.75));
    dLeftBumper.whileFalse(new SetClawStates(m_Claw, 7));
    dRightBumper.whileTrue(new RunIntake(4, -0.65));
    dRightBumper.whileFalse(new SetClawStates(m_Claw, 8));
    dKA.whileTrue(new InstantCommand(()->swerveSubsystem.zeroHeading()));
    dKY.whileTrue(new SetClawStates(m_Claw, 0));
    dKX.whileTrue(new AutoBalance(swerveSubsystem));
    // dKY.whileTrue(new SetClawStates(m_Claw, 2));
    // dKB.whileTrue(new SetClawStates(m_Claw, 3));
    // dKX.whileTrue(new SetClawStates(m_Claw, 4));
    // dLeftPov.whileTrue(new MoveArm(m_Claw, 0.5));
    // dRightPov.whileTrue(new MoveArm(m_Claw, -0.5));

    //Operator's buttons
    oLeftTrigger.whileTrue(new BottomIntake(m_Claw, 0.85, false));//Positive is Cone Intake for bottom intake
    oLeftTrigger.whileFalse(new BottomIntake(m_Claw, 0, true));
    oLeftBumper.whileTrue(new clawIntakeTester(m_Claw, -0.85)); //Positive is Outtake cone
    oLeftBumper.whileFalse(new clawIntakeHoldTester(m_Claw));
    oRightBumper.whileTrue(new clawIntakeTester(m_Claw, 0.5 )); //Negative is Outtake cube
    oRightBumper.whileFalse(new clawIntakeHoldTester(m_Claw));
    oRightTrigger.whileTrue(new BottomOuttake(m_Claw, -0.8, false));
    oRightTrigger.whileFalse(new BottomOuttake(m_Claw, 0, true));
    //oKA.whileTrue(new SetClawStates(m_Claw, 0));
    oKA.whileTrue(new ZeroModules(swerveSubsystem));
    //oKA.whileFalse(new InstantCommand(() -> swerveSubsystem.xFormation()));
    oKY.whileTrue(new SetClawStates(m_Claw, 2));
    oKB.whileTrue(new SetClawStates(m_Claw, 3));
    oKX.whileTrue(new SetClawStates(m_Claw, 4 ));
    oRightPov.whileTrue(new MoveArm(m_Claw, -0.5));
    oUpPov.whileTrue(new HandOff(0.8, false));
    oUpPov.whileFalse(new HandOff(0, true));
    oDownPov.onTrue(new testingArmExtenders(m_Claw, false));
    //SmartDashboard.putData("Auto Chooser", chooser);
    
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    dRightTrigger = driver.rightTrigger();
    dLeftTrigger = driver.leftTrigger();
    dLeftBumper = driver.leftBumper();
    dRightBumper = driver.rightBumper();
    dKA = driver.a();
    dKB = driver.b();
    dKY = driver.y();
    dKX = driver.x();
    dUpPov = driver.povUp();
    dDownPov = driver.povDown();
    dLeftPov = driver.povLeft();
    dRightPov = driver.povRight();
    
    
    oRightTrigger = operator.rightTrigger();
    oLeftTrigger = operator.leftTrigger();
    oLeftBumper = operator.leftBumper();
    oRightBumper = operator.rightBumper();
    oKA = operator.a();
    oKB = operator.b();
    oKY = operator.y();
    oKX = operator.x();
    oUpPov = operator.povUp();
    oDownPov = operator.povDown();
    oLeftPov = operator.povLeft();
    oRightPov = operator.povRight();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public void makeAuto(){
    // List<PathPlannerTrajectory> pathgroup = PathPlanner.loadPathGroup("Try3", new PathConstraints(4, 4));
    // List<PathPlannerTrajectory> tryGroup = PathPlanner.loadPathGroup("PathTesting", new PathConstraints(3, 4));
    //List<PathPlannerTrajectory> blue = PathPlanner.loadPathGroup("Blue", new PathConstraints(4, 4));
    //List<PathPlannerTrajectory> low = PathPlanner.loadPathGroup("Blue_down", new PathConstraints(4, 4));
    HashMap<String, Command> eventMap = new HashMap<>();
    //List<PathPlannerTrajectory> knock = PathPlanner.loadPathGroup("Knockout", new PathConstraints(4, 4), new PathConstraints(4, 4), new PathConstraints(4, 4), new PathConstraints(1, 1), new PathConstraints(4, 4));
    // List<PathPlannerTrajectory> topcone = PathPlanner.loadPathGroup("TopCone", new PathConstraints(0.5, 0.5), new PathConstraints(0.5, 0.5), new PathConstraints(4, 4));
    // List<PathPlannerTrajectory> topcube = PathPlanner.loadPathGroup("TopCube", new PathConstraints(0.5, 0.5), new PathConstraints(0.5, 0.5), new PathConstraints(4, 4));
    // List<PathPlannerTrajectory> conecharge = PathPlanner.loadPathGroup("ConeCharge", new PathConstraints(0.5, 0.5), new PathConstraints(0.5, 0.5), new PathConstraints(4, 4));
    // List<PathPlannerTrajectory> cubecharge = PathPlanner.loadPathGroup("CubeCharge", new PathConstraints(0.5, 0.5), new PathConstraints(0.5, 0.5), new PathConstraints(4, 4));
    // List<PathPlannerTrajectory> bottomcone = PathPlanner.loadPathGroup("BottomCone", new PathConstraints(0.5, 0.5), new PathConstraints(0.5, 0.5), new PathConstraints(4, 4));
    // List<PathPlannerTrajectory> bottomcube = PathPlanner.loadPathGroup("BottomCube", new PathConstraints(0.5, 0.5), new PathConstraints(0.5, 0.5), new PathConstraints(4, 4));
    // List<PathPlannerTrajectory> TBN = PathPlanner.loadPathGroup("NotNice", new PathConstraints(0.5, 0.5), new PathConstraints(0.5, 0.5), new PathConstraints(4, 4));
    // List<PathPlannerTrajectory> cubechargetest = PathPlanner.loadPathGroup("CubeChargeTest", new PathConstraints(0.5, 0.5), new PathConstraints(0.5, 0.5), new PathConstraints(4, 4), new PathConstraints(2, 2));

    // List<PathPlannerTrajectory> testing = PathPlanner.loadPathGroup("NotNice", new PathConstraints(0.5, 0.5), new PathConstraints(0.5, 0.5), new PathConstraints(4, 4));

    PathPlannerPath path = PathPlannerPath.fromPathFile("Testing");

    Command testPath = AutoBuilder.followPath(path);
    SequentialCommandGroup testing = new SequentialCommandGroup(new InstantCommand(() -> swerveSubsystem.resetOdometry(PathPlannerPath.fromPathFile("Testing").getPreviewStartingHolonomicPose())), testPath);

    // eventMap.put("intakecone", new RunIntake(1, 0.5));
    // eventMap.put("intakecube", new RunIntake(2, -0.5));//Fix sensor before 2nd case
    // eventMap.put("drivecone", new SetClawStates(m_Claw, 1));//Cone
    // eventMap.put("drivecube", new SetClawStates(m_Claw, 6));//Cube
    // eventMap.put("highpos", new SetClawStates(m_Claw, 2));
    // eventMap.put("midpos", new SetClawStates(m_Claw, 3));
    // eventMap.put("lowpos", new SetClawStates(m_Claw, 4));
    // eventMap.put("outtakecube", new clawIntakeTester(m_Claw, 0.75));
    // eventMap.put("outtakecone", new clawIntakeTester(m_Claw, -0.8));
    // eventMap.put("reset", new ZeroModules(swerveSubsystem));
    // eventMap.put("bottomcone", new BottomIntake(m_Claw, 0.85, false));
    // eventMap.put("stopbottomcone", new BottomIntake(m_Claw, 0, true));
    // eventMap.put("balance", new AutoBalance(swerveSubsystem));


    // //Blue Auto Paths. Do not Touch!!!
    // SwerveAutoBuilder topCone = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.000001, 0.006, 0.00001), new PIDConstants(0.001, 0.006, 0), swerveSubsystem::setModuleStates, eventMap, false, swerveSubsystem);
    // Command  blueConeTop = topCone.fullAuto(topcone);

    // SwerveAutoBuilder twoGamePieceCone = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.000001, 0.006, 0.00001), new PIDConstants(0.001, 0.006, 0), swerveSubsystem::setModuleStates, eventMap, false, swerveSubsystem);
    // Command blueTwoPiece = twoGamePieceCone.fullAuto(testing);

    // SwerveAutoBuilder topCube = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.000001, 0.006, 0.00001), new PIDConstants(0.001, 0.006, 0), swerveSubsystem::setModuleStates, eventMap, false, swerveSubsystem);
    // Command  blueCubeTop = topCube.fullAuto(topcube);

    // SwerveAutoBuilder coneCharge = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.000001, 0.006, 0.00001), new PIDConstants(0.001, 0.006, 0), swerveSubsystem::setModuleStates, eventMap, false, swerveSubsystem);
    // Command  blueConeCharge = coneCharge.fullAuto(conecharge);
    // SequentialCommandGroup BlueConeCharge = new SequentialCommandGroup(blueConeCharge, new AutoBalance(swerveSubsystem));

    // SwerveAutoBuilder cubeCharge = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.000001, 0.006, 0.00001), new PIDConstants(0.001, 0.006, 0), swerveSubsystem::setModuleStates, eventMap, false, swerveSubsystem);
    // //SequentialCommandGroup blueCubeCharge = new SequentialCommandGroup(cubeCharge.fullAuto(cubecharge), new AutoBalance(swerveSubsystem));
    // Command blueCubeCharge = cubeCharge.fullAuto(cubecharge);
    // SequentialCommandGroup BlueCubeCharge = new SequentialCommandGroup(blueCubeCharge, new AutoBalance(swerveSubsystem));

    // SwerveAutoBuilder bottomCone = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.000001, 0.006, 0.00001), new PIDConstants(0.001, 0.006, 0), swerveSubsystem::setModuleStates, eventMap, false, swerveSubsystem);
    // Command  blueConeBottom = bottomCone.fullAuto(bottomcone);

    // SwerveAutoBuilder bottomCube = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.000001, 0.006, 0.00001), new PIDConstants(0.001, 0.006, 0), swerveSubsystem::setModuleStates, eventMap, false, swerveSubsystem);
    // Command  blueCubeBottom = bottomCube.fullAuto(bottomcube);

    // SwerveAutoBuilder cubeChargeTest = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.000001, 0.006, 0.00001), new PIDConstants(0.001, 0.006, 0), swerveSubsystem::setModuleStates, eventMap, false, swerveSubsystem);
    // //SequentialCommandGroup blueCubeCharge = new SequentialCommandGroup(cubeCharge.fullAuto(cubecharge), new AutoBalance(swerveSubsystem));
    // Command blueCubeChargeTest = cubeChargeTest.fullAuto(cubechargetest);
    // SequentialCommandGroup BlueCubeChargeTest = new SequentialCommandGroup(blueCubeChargeTest, new AutoBalanceFront(swerveSubsystem));


    // //Red Auto Paths. Do not Touch!!!
    // SwerveAutoBuilder twoPieceGameCone = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.000001, 0.006, 0.00001), new PIDConstants(0.001, 0.006, 0), swerveSubsystem::setModuleStates, eventMap, true, swerveSubsystem);
    // Command redTwoPiece = twoPieceGameCone.fullAuto(testing);

    // SwerveAutoBuilder topRedCone = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.000001, 0.006, 0.00001), new PIDConstants(0.001, 0.006, 0), swerveSubsystem::setModuleStates, eventMap, true, swerveSubsystem);
    // Command  redConeTop = topRedCone.fullAuto(topcone);

    // SwerveAutoBuilder topRedCube = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.000001, 0.006, 0.00001), new PIDConstants(0.001, 0.006, 0), swerveSubsystem::setModuleStates, eventMap, true, swerveSubsystem);
    // Command  redCubeTop = topRedCube.fullAuto(topcube);

    // SwerveAutoBuilder coneRedCharge = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.000001, 0.006, 0.00001), new PIDConstants(0.001, 0.006, 0), swerveSubsystem::setModuleStates, eventMap, true, swerveSubsystem);
    // Command  redConeCharge = coneRedCharge.fullAuto(conecharge);
    // SequentialCommandGroup RedConeCharge = new SequentialCommandGroup(redConeCharge, new AutoBalance(swerveSubsystem));

    // SwerveAutoBuilder cubeRedCharge = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.000001, 0.006, 0.00001), new PIDConstants(0.001, 0.006, 0), swerveSubsystem::setModuleStates, eventMap, true, swerveSubsystem);
    // Command  redCubeCharge = cubeRedCharge.fullAuto(cubecharge);
    // SequentialCommandGroup RedCubeCharge = new SequentialCommandGroup(redCubeCharge, new AutoBalance(swerveSubsystem));

    // SwerveAutoBuilder bottomRedCone = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.000001, 0.006, 0.00001), new PIDConstants(0.001, 0.006, 0), swerveSubsystem::setModuleStates, eventMap, true, swerveSubsystem);
    // Command  redConeBottom = bottomRedCone.fullAuto(bottomcone);

    // SwerveAutoBuilder bottomRedCube = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.000001, 0.006, 0.00001), new PIDConstants(0.001, 0.006, 0), swerveSubsystem::setModuleStates, eventMap, true, swerveSubsystem);
    // Command  redCubeBottom = bottomRedCube.fullAuto(bottomcube);



    // SwerveAutoBuilder auto = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.000001, 0.006, 0.00001), new PIDConstants(0.001, 0.006, 0), swerveSubsystem::setModuleStates, eventMap, false, swerveSubsystem);
    // Command test = auto.fullAuto(testing);
    // chooser.addOption("BlueLoadZone Cone", blueConeTop);
    // chooser.addOption("BlueLoadZone Cube", blueCubeTop);
    // chooser.addOption("BlueMid Cone", BlueConeCharge);
    // chooser.addOption("BlueMid Cube", BlueCubeCharge);
    // chooser.addOption("BlueWall Cone", blueConeBottom);
    // chooser.addOption("BlueWall Cube", blueCubeBottom);
    // chooser.addOption("2 Pieces Load Zone Blue", blueTwoPiece);
    // chooser.addOption("RedLoadZone Cone", redConeTop);
    // chooser.addOption("RedLoadZone Cube", redCubeTop);
    // chooser.addOption("RedMid Cone", RedConeCharge);
    // chooser.addOption("RedMid Cube", RedCubeCharge);
    // chooser.addOption("RedWall Cone", redConeBottom);
    // chooser.addOption("RedWall Cube", redCubeBottom); 
    // chooser.addOption("2 Piece Load Zone Red", redTwoPiece);
    // chooser.addOption("Charge Cube and Taxi Test", BlueCubeChargeTest);
    chooser.addOption("TestingPath", testing);
    chooser.setDefaultOption("Nothing", null);

    SmartDashboard.putData("Auto Chooser", chooser);
    //return blueConeTop;
    //return blueCubeTop;
    //return blueConeCharge;
    //return blueCubeCharge;
    //return blueConeBottom;
    //return blueCubeBottom;
    //return redConeTop;
    //return redCubeTop;
    //return redConeCharge;
    //return redCubeCharge;
    //return redConeBottom;
    //return redCubeBottom;

  }
  public Command getAutonomousCommand() {
    
    return chooser.getSelected();

    //makeAuto();
  }
}
