// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawMechSetUp;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import javax.lang.model.util.ElementScanner14;

public class Claw extends SubsystemBase {
  public CANSparkMax leftClawArmMotor, rightClawArmMotor, bottomIntakeMotor;
  public SparkPIDController rotateArmPID;
  public AbsoluteEncoder throughBoreAbs;
  public static CANSparkMax clawIntake;
  public static boolean gamePiece;
  public static boolean isCone;
  public static boolean isCube;
  private double coneOuttakeSpd, cubeOuttakeSpd;
  public static DigitalInput clawSensor;
  public DoubleSolenoid clawExtenders, intakeArmExt, intakeRotate;
  private double groundIntakeConePos = 107;
  private double groundIntakeCubePos = 93;
  private double loadZoneIntakePosCone = 196; //10 more degrees from high peg
  private double loadZoneIntakePosCube = 189;
  private double handoff = 39;
  private double highPos = 185;
  private double midPos = 130;
  private double lowPos = 99;//switch to 60 later
  private double startConfigPos = 20;
  private double driveConfigPos = 53;
  public static boolean isFinished = false;
  private double lastArmPos;
  public SparkPIDController clawIntakePID;
  public RelativeEncoder clawIntakeEnc;
  public double intakeEncPos = 0;
  public int armStates;
  //Testing public RelativeEncoder encoderRelative = leftClawArmMotor.getEncoder();
  public static DigitalInput cylinderSensor = new DigitalInput(1);

  int newcounter = 0;

  /** Creates a new Claw. */
  public Claw() {
    coneOuttakeSpd = -0.75;
    cubeOuttakeSpd = 0.5;
    clawSensor = new DigitalInput(0);
    clawExtenders = new DoubleSolenoid(PneumaticsModuleType.REVPH, 15, 0);
    intakeArmExt = new DoubleSolenoid(PneumaticsModuleType.REVPH, 14, 1);
    intakeRotate = new DoubleSolenoid(PneumaticsModuleType.REVPH, 13, 2);
    leftClawArmMotor = new CANSparkMax(11, MotorType.kBrushless);
    rightClawArmMotor = new CANSparkMax(12, MotorType.kBrushless);
    bottomIntakeMotor = new CANSparkMax(14, MotorType.kBrushless);
    clawIntake = new CANSparkMax(13, MotorType.kBrushless);

    leftClawArmMotor.restoreFactoryDefaults();
    rightClawArmMotor.restoreFactoryDefaults();
    clawIntake.restoreFactoryDefaults();
    bottomIntakeMotor.restoreFactoryDefaults();

    rotateArmPID = leftClawArmMotor.getPIDController();
    clawIntakePID = clawIntake.getPIDController();
    throughBoreAbs = leftClawArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
    clawIntakeEnc = clawIntake.getEncoder();

    rotateArmPID.setFeedbackDevice(throughBoreAbs);
    rotateArmPID.setP(0.006);
    rotateArmPID.setI(0);
    rotateArmPID.setD(0);

    clawIntakePID.setFeedbackDevice(clawIntakeEnc);
    clawIntakePID.setP(0.008);
    clawIntakePID.setI(0);
    clawIntakePID.setD(0);

    leftClawArmMotor.setIdleMode(IdleMode.kBrake);
    rightClawArmMotor.setIdleMode(IdleMode.kBrake);
    bottomIntakeMotor.setIdleMode(IdleMode.kCoast);

    leftClawArmMotor.setSmartCurrentLimit(40);
    rightClawArmMotor.setSmartCurrentLimit(40);
    bottomIntakeMotor.setSmartCurrentLimit(40);
    clawIntake.setSmartCurrentLimit(40);

    throughBoreAbs.setPositionConversionFactor(360);
    leftClawArmMotor.setInverted(true);
    rightClawArmMotor.setInverted(false);
    throughBoreAbs.setInverted(true);

    leftClawArmMotor.burnFlash();
    rightClawArmMotor.burnFlash();
    clawIntake.burnFlash();
    bottomIntakeMotor.burnFlash();
    clawExtenders.set(Value.kReverse);

    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("Absolute Position " + throughBoreAbs.getPosition());
    System.out.println("Mag Switch: " + cylinderSensor.get());
    // System.out.println("Infrared Sensor: " + clawSensor.get());
    //System.out.println("Test Relative Encoder" );
  }

  
  //Intake Positions
  public void intakeStates(int intakeState, double speed, int counter){
    switch (intakeState){
      //Default
      case 0: 
        clawIntake.set(0);
        clawExtenders.set(Value.kOff);
        break;

      //Cone Ground Intake
      case 1:
      rotateArmPID.setP(0.006);
      rotateArmPID.setReference(groundIntakeConePos, ControlType.kPosition);
      rightClawArmMotor.follow(leftClawArmMotor, true);
      if(throughBoreAbs.getPosition() >= (groundIntakeConePos - 20)){
        clawExtenders.set(Value.kForward);
        clawIntake.set(speed);
      }
      // if (clawSensor.get() == true){
      // //Rotate the arm to the ground position
      //   rotateArmPID.setReference(groundIntakeConePos, ControlType.kPosition);
      //   rightClawArmMotor.follow(leftClawArmMotor, true);

      //   //If the arm is at the groud position and the claw sensor doesn't see anything extend pneumatics and run cone intake
      //   if(throughBoreAbs.getPosition() >= (groundIntakeConePos - 20)){
      //     clawExtenders.set(Value.kReverse);
      //     clawIntake.set(speed);
      //   }
      // }
      // else
      // {
      //   clawExtenders.set(Value.kForward);
      //   clawIntake.set(-0.3);
      //   if (cylinderSensor.get() == true){
      //     counter = 0;
      //     if(counter > 150){
      //       clawIntake.set(0);
      //       rotateArmPID.setReference(driveConfigPos, ControlType.kPosition);
      //       rightClawArmMotor.follow(leftClawArmMotor, true);
      //     }  

      // }
      // }

        break;

      //Cube Ground Intake
      case 2:
      rotateArmPID.setP(0.006);
      rotateArmPID.setReference(groundIntakeCubePos, ControlType.kPosition);
      rightClawArmMotor.follow(leftClawArmMotor, true);
      if(throughBoreAbs.getPosition() >= (groundIntakeCubePos - 20)){
        clawExtenders.set(Value.kForward);
        clawIntake.set(speed);
      }
      // rotateArmPID.setP(0.006);

      // if (clawSensor.get() == true)
      // {
      // //Rotate the arm to the ground position
      //   rotateArmPID.setReference(groundIntakeCubePos, ControlType.kPosition);
      //   rightClawArmMotor.follow(leftClawArmMotor, true);

      //   //If the arm is at the groud position and the claw sensor doesn't see anything extend pneumatics and run cone intake
      //   if(throughBoreAbs.getPosition() >= (groundIntakeCubePos - 20)){
      //     clawExtenders.set(Value.kReverse);
      //     clawIntake.set(speed);
      //   }
      // }
      // else
      // {
      //   clawExtenders.set(Value.kReverse);
      //   clawIntake.set(0.1);
      //   if (cylinderSensor.get() == true){
      //     counter = 0;
      //     if(counter > 150){
      //       clawIntake.set(0);
      //       rotateArmPID.setReference(driveConfigPos, ControlType.kPosition);
      //       rightClawArmMotor.follow(leftClawArmMotor, true);
      //     }
      //   }
      // }
        break;

      //Loading Zone Cone Intake
      case 3:
      rotateArmPID.setP(0.006);
      if (clawSensor.get() == true)
      {
      //Rotate the arm to the ground position
        rotateArmPID.setReference(loadZoneIntakePosCone, ControlType.kPosition);
        rightClawArmMotor.follow(leftClawArmMotor, true);

        //If the arm is at the groud position and the claw sensor doesn't see anything extend pneumatics and run cone intake
        if(throughBoreAbs.getPosition() >= (loadZoneIntakePosCone - 20)){
          clawIntake.set(speed);
          System.out.println("Picking up cone!");
          //intakeEncPos = clawIntakeEnc.getPosition();
        }
        newcounter = 0;
      }
      else
      {
        newcounter++;
        clawExtenders.set(Value.kReverse);
        // clawIntake.set(-0.1);
        System.out.println("mag = " + cylinderSensor.get() + "counter = " + newcounter);
      //    if(newcounter < 300){
      //  //    clawIntake.set(0);
      //  System.out.println("delay!");
      //       clawIntake.set(-.1);
            clawIntake.set(-0.2);
      //   }

        
      }

        break;
      // //Rotate the arm to the loading zone position
      //   rotateArmPID.setReference(loadZoneIntakePos, ControlType.kPosition);
      //   //If the arm is in the loading zone position and the sensor reads false, then do not extend pneumatics and run intake for cone
      //   if(throughBoreAbs.getPosition() == loadZoneIntakePos && clawSensor.get() == false){
      //     clawExtenders.set(Value.kReverse);
      //     clawIntake.set(speed);
      //     intakeEncPos = clawIntakeEnc.getPosition();
      //   }
      //   //If the sensor does see a game piece, for this case it is a cone, so set booleans to true except cube boolean
      //   else if(clawSensor.get() == true){
      //     gamePiece = true;
      //     isCone = true;
      //     isCube = false;
      //     if(cylinderSensor.get() == true){
      //       rotateArmPID.setReference(driveConfigPos, ControlType.kPosition);
      //       rightClawArmMotor.follow(leftClawArmMotor, true);
      //     }
          
      //   }
      
      //Loading Zone Cube Intake
      case 4:
      rotateArmPID.setP(0.006);
      if (clawSensor.get() == true)
      {
      //Rotate the arm to the ground position
        rotateArmPID.setReference(loadZoneIntakePosCube, ControlType.kPosition);
        rightClawArmMotor.follow(leftClawArmMotor, true);

        //If the arm is at the groud position and the claw sensor doesn't see anything extend pneumatics and run cone intake
        if(throughBoreAbs.getPosition() >= (loadZoneIntakePosCube - 20)){
          clawIntake.set(speed);
          System.out.println("Picking up cube!");
          //intakeEncPos = clawIntakeEnc.getPosition();
        }
      }
      else
      {
        counter++;
        clawExtenders.set(Value.kReverse);
        clawIntake.set(0.1);
        System.out.println("mag = " + cylinderSensor.get());
      }
        break;
      //Rotate the arm to the loading zone position
        // rotateArmPID.setReference(loadZoneIntakePos, ControlType.kPosition);
        // //If the arm is in the loading zone position and the sensor reads false, then do not extend pneumatics and run intake for cube
        // if(throughBoreAbs.getPosition() == loadZoneIntakePos && clawSensor.get() == false){
        //   clawExtenders.set(Value.kReverse);
        //   clawIntake.set(speed);
        // }
        // //If the sensor does see a game piece, for this case it is a cube, so set booleans to true except cone boolean.
        // else if(clawSensor.get() == true){
        //   gamePiece = true;
        //   isCone = false;
        //   isCube = true;
        //   if(cylinderSensor.get() == true){
        //     rotateArmPID.setReference(driveConfigPos, ControlType.kPosition);
        //     rightClawArmMotor.follow(leftClawArmMotor, true);
        //   }
          
        // }
        // break;

      //Autonomus Cases. Do not touch!!!
      case 5:
        rotateArmPID.setP(0.006);
        rotateArmPID.setReference(groundIntakeCubePos, ControlType.kPosition);
        rightClawArmMotor.follow(leftClawArmMotor, true);

      //If the arm is at the groud position and the claw sensor doesn't see anything extend pneumatics and run cube intake
        if(throughBoreAbs.getPosition() >= (groundIntakeCubePos - 20)){
          clawExtenders.set(Value.kForward);
          clawIntake.set(speed);
          if(clawSensor.get() == false){
            clawIntake.set(0);
          }
          
        }
        break;
    }
  }


  //Arm Positions
  public void setClawStates(int clawStates, int counter){

    clawExtenders.set(Value.kReverse);
    switch(clawStates){
      //Starting config state
      case 0:
        rotateArmPID.setP(0.003);
        rotateArmPID.setReference(startConfigPos, ControlType.kPosition);
        rightClawArmMotor.follow(leftClawArmMotor, true);
        clawIntake.set(0);
        clawExtenders.set(Value.kReverse);
        isFinished = false;
        gamePiece = false;
        isCone = false;
        isCube = false;
        break;
      //Drive config state
      case 1:
        rotateArmPID.setP(0.003);
        clawExtenders.set(Value.kReverse);
        if(counter > 50){
          rotateArmPID.setReference(startConfigPos, ControlType.kPosition);
          rightClawArmMotor.follow(leftClawArmMotor, true);
          clawIntake.set(0.1);
        }
        // clawIntake.set(-0.02);
        break;

      //Set high arm position used for auto claw
      case 2:
        rotateArmPID.setP(0.005);
        rotateArmPID.setReference(highPos, ControlType.kPosition);
        rightClawArmMotor.follow(leftClawArmMotor, true);
        //clawIntakePID.setReference(intakeEncPos, ControlType.kPosition);
        break;

      //Set mid arm position used for auto claw
      case 3:
        rotateArmPID.setP(0.006);
        rotateArmPID.setReference(midPos, ControlType.kPosition);
        rightClawArmMotor.follow(leftClawArmMotor, true);
        //clawIntakePID.setReference(intakeEncPos, ControlType.kPosition);
        break;

      //Set low arm position used for auto claw
      case 4:
        rotateArmPID.setReference(lowPos, ControlType.kPosition);
        rightClawArmMotor.follow(leftClawArmMotor, true);
        if(throughBoreAbs.getPosition() >= 85){
          clawExtenders.set(Value.kForward);
        }
        //clawIntakePID.setReference(intakeEncPos, ControlType.kPosition);
        break;

      //Autonomus Cases Do not touch!!!
      case 5:
        //Stop the intake and if cylinders are in return to driving config
        rotateArmPID.setP(0.006);
        clawIntake.set(0);
        if(cylinderSensor.get() == true){
          rotateArmPID.setReference(startConfigPos, ControlType.kPosition);
          rightClawArmMotor.follow(leftClawArmMotor, true);
        }
        break;
      //Driving configuration after scroring in autonomous
      case 6:
          clawIntake.set(0);
          rotateArmPID.setReference(startConfigPos, ControlType.kPosition);
          rightClawArmMotor.follow(leftClawArmMotor, true);
        break;
      
      case 7:
        rotateArmPID.setP(0.003);
        rotateArmPID.setReference(startConfigPos, ControlType.kPosition);
        rightClawArmMotor.follow(leftClawArmMotor, true);
        clawIntake.set(0.1);
        // clawIntake.set(-0.02);
        break;

      case 8:
        rotateArmPID.setP(0.003);
        rotateArmPID.setReference(startConfigPos, ControlType.kPosition);
        rightClawArmMotor.follow(leftClawArmMotor, true);
        //clawIntake.set(0.1);
        // clawIntake.set(-0.02);
        break;
      
      
    }
  }
  //To set the outtake speed with one button 
  // public void setOuttakeSpd(){
  //   if(isCone == true && isCube == false){
  //     clawIntake.set(coneOuttakeSpd);
  //     if(clawSensor.get() == false){
  //       clawIntake.set(0);
  //       isCone = false;
  //       isFinished = true;
  //     }
  //   }
  //   else if(isCone == false && isCube == true){
  //     clawIntake.set(cubeOuttakeSpd);
  //     if(clawSensor.get() == false){
  //       clawIntake.set(0);
  //       isCube = false;
  //     }
  //   }
  // }

  //Needed for auto claw
  public boolean getGamePiece(){
    return gamePiece;
  }
  //Needed for auto vision
  public static boolean getIsCone(){
    return isCone;
  }
  //Needed for auto vision
  public static boolean getIsCube(){
    return isCube;
  }
  
  //Override functions
  public void forward(){
    System.out.println("Mag: " + cylinderSensor.get());
    clawExtenders.set(Value.kForward);
  }
  public void reverse(){
    System.out.println("Mag: " + cylinderSensor.get());
    clawExtenders.set(Value.kReverse);
  }
  public void off(){
    clawExtenders.set(Value.kOff);
  }
  // public void moveArm(double speed){
  //   // if(!cylinderSensor.get()){
  //   //   leftClawArmMotor.set(speed);
  //   //   rightClawArmMotor.set(speed);
  //   //   lastArmPos = throughBoreAbs.getPosition();
  //   // }
  //   System.out.println("Limit Switch is Pressed: " + cylinderSensor.get());
  //   // else{
  //   //   leftClawArmMotor.set(0);
  //   //   rightClawArmMotor.set(0);
  //   // }
  // }
  public void intakeTester(double speed){
    clawIntake.set(speed);
    
  }
  public void intakeHold(){
    clawIntakePID.setReference(clawIntakeEnc.getPosition(), ControlType.kPosition);
  }
  public void holdArm(){
    rotateArmPID.setReference(lastArmPos, ControlType.kPosition);
    rightClawArmMotor.follow(leftClawArmMotor, true);
  }
  public void setArmState(int armState){
    armStates = armState;
  }

  public void armMove(double speed){
    if(cylinderSensor.get() == true){
      leftClawArmMotor.set(speed);
      rightClawArmMotor.set(speed);
    }
  }

  public void stopIntakes(){
    intakeArmExt.set(Value.kReverse);
  }

  //This is to run the bottom intake
  public void bottomIntake(double speed, int counter) {
    rotateArmPID.setReference(handoff, ControlType.kPosition);
    rightClawArmMotor.follow(leftClawArmMotor, true);
    intakeArmExt.set(Value.kForward);
    if(counter > 50){
      intakeRotate.set(Value.kForward);
      bottomIntakeMotor.set(speed);
    }
  }

  //This is to return the bottom intake into the stationary position
  public void returnBottomIntake(double speed, int counter) {
    bottomIntakeMotor.set(speed);
    intakeRotate.set(Value.kReverse);
    intakeArmExt.set(Value.kReverse);
  }

  //This is Outtake command for the bottom intake
  public void bottomOuttake(double speed, int counter){
    intakeArmExt.set(Value.kForward);
    if(counter > 50){
      intakeRotate.set(Value.kForward);
      bottomIntakeMotor.set(speed);
    }
  }

  public void handOffPosition(double speed, int counter){
        clawIntake.set(speed);
        rotateArmPID.setReference(handoff, ControlType.kPosition);
        rightClawArmMotor.follow(leftClawArmMotor, true);
        if(throughBoreAbs.getPosition() >= (handoff - 20) && counter > 50){
          bottomIntakeMotor.set(-0.95);
  }
}

  public void stopHandOff(double speed, int counter){
    clawIntake.set(speed);
    rotateArmPID.setReference(startConfigPos, ControlType.kPosition);
    rightClawArmMotor.follow(leftClawArmMotor, true);
    bottomIntakeMotor.set(speed);
  }
}

