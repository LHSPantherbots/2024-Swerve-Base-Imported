// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkBase.SoftLimitDirection;
// import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {

  private final SparkMax m_RightArm;
  private final SparkMax m_LeftArm;
  private final SparkMaxConfig c_RightArm;
  private final SparkMaxConfig c_LeftArm;


  RelativeEncoder e_RightArm;
  RelativeEncoder e_LeftArm;

  private double kP = 0.05;
  private double kI = 0.0;
  private double kD = 0.0;
  private double maxVel = 90.0;
  private double maxAcc = 90.0;
  private double allowableError = 1.0;
  private double heightSetpoint = 0.0;
  private static double kDt = 0.02;
  private final TrapezoidProfile.Constraints m_constraints;
  private final ProfiledPIDController m_controller;

  /** Creates a new ElevatorSubsystem. */
  public Climb() {

    m_RightArm = new SparkMax(ClimbConstants.kArmRight, MotorType.kBrushless);
    m_LeftArm = new SparkMax(ClimbConstants.kArmLeft, MotorType.kBrushless);
    c_RightArm = new SparkMaxConfig();
    c_LeftArm = new SparkMaxConfig();

    c_RightArm
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(60)
      .closedLoopRampRate(0.25);
    c_RightArm.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(175)
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(0);
    c_LeftArm
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(60)
      .closedLoopRampRate(0.25);
    c_LeftArm.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(175)
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(0);

    m_RightArm.configure(c_RightArm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_LeftArm.configure(c_LeftArm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    e_RightArm = m_RightArm.getEncoder();
      e_LeftArm = m_LeftArm.getEncoder();

      m_constraints = new TrapezoidProfile.Constraints(maxVel, maxAcc);
      m_controller = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);

    e_LeftArm.setPosition(0);
    e_RightArm.setPosition(0);

  }

  @Override
  public void periodic() {
    // Smart Dashboard Items
    SmartDashboard.putNumber("Right Climber Height", getRightHeight());
    SmartDashboard.putNumber("Left Climber Height", getLeftHeight());
    SmartDashboard.putBoolean("Right Climber at Set Height", isAtHeightRight());
    SmartDashboard.putBoolean("Left Climber at Set Height", isAtHeightLeft());
    SmartDashboard.putNumber("Climber Height Setpoint", getHeightSetpoint());
    SmartDashboard.putNumber("Right Climb Arm Velocity", e_RightArm.getVelocity());
    SmartDashboard.putNumber("Left Climb Arm Velocity", e_LeftArm.getVelocity());
    SmartDashboard.putNumber("Right Climber Amps", m_RightArm.getOutputCurrent());
    SmartDashboard.putNumber("Left Climber Amps", m_LeftArm.getOutputCurrent());

  }

  public double getRightHeight() {
    return e_RightArm.getPosition();
  }

  public double getLeftHeight() {
    return e_LeftArm.getPosition();
  }

  public boolean isAtHeightRight() {
    double error = getRightHeight() - heightSetpoint;
    return (Math.abs(error) < allowableError);
  }

  public boolean isAtHeightLeft() {
    double error = getLeftHeight() - heightSetpoint;
    return (Math.abs(error) < allowableError);
  }

  public double getHeightSetpoint() {
    return heightSetpoint;
  }

  public void setHeightSetpoint(double setPoint) {
    heightSetpoint = setPoint;
  }

  public void manualRightArm(double move) {
    m_RightArm.set(move);
    // if ((getRightHeight() < ClimbConstants.heightUpperLimit || move < 0) &&
    // (getRightHeight() > ClimbConstants.heightLowerLimit || move > 0)) {
    // m_RightArm.set(move);
    // } else {
    // m_RightArm.set(0);
    // }
  }

  public void manualLeftArm(double move) {
    m_LeftArm.set(move);
    // if ((getLeftHeight() < ClimbConstants.heightUpperLimit || move < 0) &&
    // (getLeftHeight() > ClimbConstants.heightLowerLimit || move > 0)) {
    // m_LeftArm.set(move);
    // } else {
    // m_LeftArm.set(0);
    // }
  }

  public void manualAll(double lift, double adjust) {
    manualRightArm(lift + adjust);
    manualLeftArm(lift - adjust);
  }

  public void resetController() {
    m_controller.reset(e_RightArm.getPosition());
    m_controller.reset(e_LeftArm.getPosition());

  }

  public void stopRightArm() {
    m_RightArm.set(0.0);
  }

  public void stopLeftArm() {
    m_LeftArm.set(0.0);
  }

  public void stopAll() {
    stopRightArm();
    stopLeftArm();
  }

  public void closedLoopElevator() {
    m_RightArm.set(m_controller.calculate(e_RightArm.getPosition(), heightSetpoint));
    m_LeftArm.set(m_controller.calculate(e_LeftArm.getPosition(), heightSetpoint));
  }

}