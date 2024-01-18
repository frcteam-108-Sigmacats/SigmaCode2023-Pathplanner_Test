// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
    public static final class SwerveConstants{
        //Used to prevent stick drift
        public static double deadband = 0.15;

        //gear ratios for motors
        public static double driveGearRat = (4.71 / 1);
        public static double turnGearRat = (4.71 / 1);

        //Module Measurements
        public static final double wheelDiameter = Units.inchesToMeters(3);
        public static double wheelCircumference = wheelDiameter * Math.PI;

        //Conversion Calculations
        public static final double kDriveEncoderRot2Meters = (Math.PI * wheelDiameter) / driveGearRat;
        public static final double kTurnEncoderRot2Rad = 2 * Math.PI;//radians 
        public static final double kDriveEncoderRPM2MPS = kDriveEncoderRot2Meters / 60;//,eters per second
        public static final double kTurnEncoderRPM2RadPerSec = kTurnEncoderRot2Rad / 60;//radians per second

        public static final double kTurnEncPosPIDMinOutput = 0; //radians
        public static final double kTurnEncPosPIDMaxOutput = kTurnEncoderRot2Rad;//radians

        public static final double kDrivingMotorFreeSpeedRps = 5676 / 60;

        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * wheelCircumference)
        / driveGearRat;

        //Used to set up speed for both drive and turn motors
        public static double maxDriveSpeed = 12;
        public static double autoMaxDriveSpeed = 3.0;
        public static double maxTurnSpeed = 2 * Math.PI;

        //Distance between the front and back wheel
        public static double wheelBase = (24.5);
        //Distance between the left and right wheel
        public static double trackWidth = (21.625);

        public static double wheelBaseMeters = Units.inchesToMeters(24.5);
        public static double trakcWidthMeters = Units.inchesToMeters(21.625);

        //Sets the current for the speed controllers. Units: Amps
        public static final int driveCurrent = 40;
        public static final int turnCurrent = 30;

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(new Translation2d(wheelBase / 2.0, trackWidth / 2.0)
        , new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), 
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        public static Translation2d flOffset = new Translation2d(wheelBaseMeters, trakcWidthMeters);
    }
    public static final class SwerveSetUp{
        //Module 1 Constants
            //Motor Set Up
            public static final int fLDriveMotor = 1;
            public static final int fLTurnMotor = 2;
            public static final boolean isFLDriveMotorReversed = true;
            public static final boolean isFLTurnMotorReversed = false;
            //Encoder Set Up
            //public static final int fLAbsoluteEnc = 0;
            public static final double fLChassisAngleOffset = 1.571;//Module relative to the chassis
            public static final double xFormOne = 45;
            //PID Controller Set Up
            public static final double frontLDriveKP = 0.2;
            public static final double frontLDriveKI = 0;
            public static final double frontLDriveKD = 0;
            public static final double frontLTurnKP = 0.8;
            public static final double frontLTurnKI = 0;
            public static final double frontLTurnKD = 0.0;

        //Module 2 Constants
            //Motor Set Up
            public static final int fRDriveMotor = 3;
            public static final int fRTurnMotor = 4;
            public static final boolean isFRDriveMotorReversed = false;
            public static final boolean isFRTurnMotorReversed = false;
            //Absolute Encoder Set Up
            //public static final int fRAbsoluteEnc = 1;
            public static final double fRChassisAngleOffset = 0;
            public static final double xFormTwo = 135;
            //PID Controller Set Up
            public static final double frontRDriveKP = 0.2;
            public static final double frontRDriveKI = 0;
            public static final double frontRDriveKD = 0;
            public static final double frontRTurnKP = 0.8;
            public static final double frontRTurnKI = 0;
            public static final double frontRTurnKD = 0;

        //Module 3 Constants
            //Drive Motor Set Up
            public static final int bLDriveMotor = 5;
            public static final int bLTurnMotor = 6;
            public static final boolean isBLDriveMotorReversed = true;
            public static final boolean isBLTurnMotorReversed = false;
            //Absolute Encoder Set Up
            //public static final int bLAbsoluteEnc = 2;
            public static final double bLChassisAngleOffset = 0;
            public static final double xFormThree = 135;
            //PID Controller Set Up
            public static final double backLDriveKP = 0.2;
            public static final double backLDriveKI = 0;
            public static final double backLDriveKD = 0;
            public static final double backLTurnKP = 0.8;
            public static final double backLTurnKI = 0;
            public static final double backLTurnKD = 0;

        //Module 4 Constants
            //Motor Set Up
            public static final int bRDriveMotor = 7;
            public static final int bRTurnMotor = 8;
            public static final boolean isBRDriveMotorReversed = false;
            public static final boolean isBRTurnMotorReversed = false;
            //Absolute Encoder Set Up
            //public static final int bRAbsoluteEnc = 3;

            public static final double bRChassisAngleOffset = Math.PI / 2;//Module relative to the chassis
            public static final double xFormFour = 45;
            //PID Controller Set Up
            public static final double backRDriveKP = 0.2;
            public static final double backRDriveKI = 0;
            public static final double backRDriveKD = 0;
            public static final double backRTurnKP = 0.8;
            public static final double backRTurnKI = 0;
            public static final double backRTurnKD = 0;


            

        //Gyro Set Up
        public static boolean invertedGyro = false;
    }

    public static final class ClawMechSetUp{
        public static final int clawIntake = 11;
        public static final int infraSensor = 0;
    }

}
