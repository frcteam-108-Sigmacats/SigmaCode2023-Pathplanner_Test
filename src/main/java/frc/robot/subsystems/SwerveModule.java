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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
  private SwerveModuleState mDesiredState = new SwerveModuleState();
  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;

  private SparkPIDController drivePID;
  private SparkPIDController turningPID;

  private RelativeEncoder driveEncoder;
  private AbsoluteEncoder turnEncoder;
  private double chassisAngleOffset, autoAngle;

  private SwerveModuleState desiredState = new SwerveModuleState();
  /** Creates a new ExampleSubsystem. */
  public SwerveModule(int driveMotorID, boolean driveMotorReversed, double kDP, double kDI, double kDD, 
  int turnMotorID, boolean turnMotorReversed, double kTP, double kTI, double kTD, double absolutePositionOffset) {
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

    //Factory reset to configure the speed controllers. Useful for when swapping out speed controllers
    driveMotor.restoreFactoryDefaults();
    turnMotor.restoreFactoryDefaults();

    //Set up of encoders and PID controllers
    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
    drivePID = driveMotor.getPIDController();
    turningPID = turnMotor.getPIDController();
    drivePID.setFeedbackDevice(driveEncoder);
    turningPID.setFeedbackDevice(turnEncoder);

    turningPID.setPositionPIDWrappingEnabled(true);
    turningPID.setPositionPIDWrappingMinInput(SwerveConstants.kTurnEncPosPIDMinOutput);
    turningPID.setPositionPIDWrappingMaxInput(SwerveConstants.kTurnEncPosPIDMaxOutput);

    //Modules angle relative to the chassis
    chassisAngleOffset = absolutePositionOffset;
    autoAngle = 45;

    turnEncoder.setInverted(true);

    drivePID.setP(kDP);
    drivePID.setI(kDI);
    drivePID.setD(kDD);
    drivePID.setFF(1 / SwerveConstants.kDriveWheelFreeSpeedRps);
    drivePID.setOutputRange(-1, 1);

    turningPID.setP(kTP);
    turningPID.setI(kTI);
    turningPID.setD(kTD);
    turningPID.setFF(0);
    turningPID.setOutputRange(-1, 1);

    //Setting motors to brake mode to prevent easy movements of the motors when the robot is on
    driveMotor.setIdleMode(IdleMode.kBrake);
    turnMotor.setIdleMode(IdleMode.kBrake);

    driveMotor.setInverted(driveMotorReversed);
    turnMotor.setInverted(turnMotorReversed);

    //Setting current limit to make sure the Spark Max's do not burn out Unit:AMPS
    driveMotor.setSmartCurrentLimit(SwerveConstants.driveCurrent);
    turnMotor.setSmartCurrentLimit(SwerveConstants.turnCurrent);

    //Applying conversion factors for position and velocity
    driveEncoder.setPositionConversionFactor(SwerveConstants.kDriveEncoderRot2Meters);
    driveEncoder.setVelocityConversionFactor(SwerveConstants.kDriveEncoderRPM2MPS);
    turnEncoder.setPositionConversionFactor(SwerveConstants.kTurnEncoderRot2Rad);
    turnEncoder.setVelocityConversionFactor(SwerveConstants.kTurnEncoderRPM2RadPerSec);

    driveMotor.burnFlash();
    turnMotor.burnFlash();

    chassisAngleOffset = absolutePositionOffset;
    driveEncoder.setPosition(0);
    desiredState.angle = new Rotation2d(turnEncoder.getPosition());
  }

  //Getting each Modules states velocity and angular position with encoders
  public SwerveModuleState getState(){
    return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition() - chassisAngleOffset));
  }

  //Getting each modules position. Necessary for odometry appliance
  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(turnEncoder.getPosition() - chassisAngleOffset));
  }

  //Setting modules desired state
  public void setDesiredState(SwerveModuleState desiredState){
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    mDesiredState = desiredState;
    correctedDesiredState.speedMetersPerSecond = mDesiredState.speedMetersPerSecond;
    correctedDesiredState.angle = mDesiredState.angle.plus(Rotation2d.fromRadians(chassisAngleOffset));

    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(turnEncoder.getPosition()));

    drivePID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    this.desiredState = desiredState;
    
  }


  //Resetting drive motors encoders
  public void resetEncoder(){
    driveEncoder.setPosition(0);
  }
  public void zeroModules(){
    mDesiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(chassisAngleOffset));
    turningPID.setReference(mDesiredState.angle.getRadians(), ControlType.kPosition);
  }

  public void setModAngle(double angle){
    mDesiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(angle));
    turningPID.setReference(mDesiredState.angle.getRadians(), ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
