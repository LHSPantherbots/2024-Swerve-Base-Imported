// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.5;//4.8 * 2;
    public static final double kMaxAngularSpeed = 2.5 * Math.PI; //2*pi radians per second

    public static final double kDirectionSlewRate = 5.0;//1.2; // radians per second
    public static final double kMagnitudeSlewRate = 5.0;//1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 5.0;//4.0;//2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(20);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = 0;
    public static final double kBackRightChassisAngularOffset = 0;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final int kPigeonCAN_Id = 30;
    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {


    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the Swerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 4 * 25.4 / 1000.0; // 4" wheel
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    // public static final double kDrivingMotorReduction = (42/12) * (18/28) * (45/15);

    public static final double kDrivingMotorReduction = 6.75;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = .04;
    // public static final double kDrivingP = 0.3;
    public static final double kDrivingI = 0.0;//0.1;
    public static final double kDrivingD = 0.0;//0.1;
    public static final double kDrivingFF = 1.0 / kDriveWheelFreeSpeedRps;
    //public static final double kDrivingFF = 0.25 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 50; // amps
  }

  public static final class IntakeConstants{
    public static final int kFrontIntake = 20;
    public static final int kBackIntake = 21;
  }

  public static final class LauncherConstants{
    public static final int kLauncherTop = 33;
    public static final int kLauncherBottom = 31;
    public static final int kFeeder = 32;
  }

  public static final class ClimbConstants{
    public static final int kArmLeft = 40;
    public static final int kArmRight = 41;
    public static final int heightUpperLimit = 415; //TODO NOT CLARIFIED AS THE LIMIT NUMBER, WILL NEED TO BE CHANGED
    public static final int heightLowerLimit = 0; //TODO NOT CLARIFIED AS THE LIMIT NUMBER, WILL NEED TO BE CHANGED
  }
  
  public static final class FulcrumConstants{
    public static final int kFulcrumRight = 50;
    public static final int kFulcrumLeft = 51;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;
    public static final int kOperatorControllerPort = 1;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class RIO_Channels_DIO {
    public static final int FEEDER_BEAM_BREAK = 0;
  }

  public static final class LEDs{
    public static final int purple_Red = 148;
    public static final int purple_Green = 0;
    public static final int purple_Blue = 211;
    public static final int purple_Hue = 282;
    public static final int purple_Sat = 100;
    public static final int purple_Val = 83;

    public static final int yellow_Red = 255;
    public static final int yellow_Green = 90;
    public static final int yellow_Blue = 0;
    public static final int yellow_Hue = 21;
    public static final int yellow_Sat = 100;
    public static final int yellow_Val = 100;

    public static final int orange_Red = 251;
    public static final int orange_Green = 51;
    public static final int orange_Blue = 0;
    public static final int orange_Hue = 12;
    public static final int orange_Sat = 100;
    public static final int orange_Val = 98;

    public static final int red_Hue = 0;
    public static final int red_Sat = 100;
    public static final int red_Val = 100;

    public static final int blue_Hue = 240;
    public static final int blue_Sat = 100;
    public static final int blue_Val = 100;

    public static final int green_Hue = 240;
    public static final int green_Sat = 100;
    public static final int green_Val = 100;

    public static final int Red = 255;
    public static final int Green = 255;
    public static final int Blue = 255;
    public static final int white_Hue = 0;
    public static final int white_Sat = 0;
    public static final int white_Val = 100;
  }
}
