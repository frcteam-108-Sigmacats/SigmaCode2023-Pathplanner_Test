// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  public NetworkTable limeLight = NetworkTableInstance.getDefault().getTable("limelight");
  public double tx, ty, ta;
  /** Creates a new Vision. */
  public Vision() {
    limeLight.getEntry("ledMode").setNumber(1);
    limeLight.getEntry("stream").setNumber(1);
    CameraServer.startAutomaticCapture();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getXValue();
    getYValue();
    getAValue();
  }
  public void getXValue(){
    tx = limeLight.getEntry("ty").getDouble(0);
  }
  public void getYValue(){
    ty = limeLight.getEntry("tx").getDouble(0);
  }
  public void getAValue(){
    ta = limeLight.getEntry("ta").getDouble(0);
  }
}
