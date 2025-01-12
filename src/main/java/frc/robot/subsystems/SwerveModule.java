// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.REVPhysicsSim; TODO: MIGRATE TO WPILIB SIM
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants.ModuleConstants;

// import frc.utils.SparkPIDWrapper;
// import frc.utils.SparkMax;

public class SwerveModule {
  private final SparkMax m_driving;
  private final SparkMax m_turning;
  private final SparkMaxConfig c_driving;
  private final SparkMaxConfig c_turning;

  private final RelativeEncoder e_driving;
  private final AbsoluteEncoder e_turning;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a SwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. 
   */
  public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    
    m_driving = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turning = new SparkMax(turningCANId, MotorType.kBrushless);
    c_driving = new SparkMaxConfig();
    c_turning = new SparkMaxConfig();

    c_turning
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(50);
    c_turning.encoder
      .inverted(true)
      .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
      .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    c_turning.closedLoop
      .positionWrappingEnabled(true)
      .positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput)
      .positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput)
      .pidf(ModuleConstants.kTurningP,ModuleConstants.kTurningI,ModuleConstants.kTurningD,ModuleConstants.kTurningFF)
      .outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);


    c_driving
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(50);
    c_driving.encoder
    .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
    .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor)
    .quadratureMeasurementPeriod(16);
    c_driving.absoluteEncoder
      .averageDepth(2);
    c_driving.closedLoop
      .pidf(ModuleConstants.kDrivingP,ModuleConstants.kDrivingI,ModuleConstants.kDrivingD,ModuleConstants.kDrivingFF)
      .outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    m_driving.configure(c_driving,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    m_turning.configure(c_turning,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    
    


    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    e_turning = m_turning.getAbsoluteEncoder();
    e_driving = m_driving.getEncoder();



    m_drivingPIDController = m_driving.getClosedLoopController();
    m_turningPIDController = m_turning.getClosedLoopController();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(e_turning.getPosition());
    e_driving.setPosition(0);
    if (RobotBase.isSimulation()) {
      //REVPhysicsSim.getInstance().addSparkMax(m_driving, DCMotor.getNEO(1));
      //REVPhysicsSim.getInstance().addSparkMax(m_turning, DCMotor.getNEO(1));
    }
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(e_driving.getVelocity(),
        new Rotation2d(e_turning.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        e_driving.getPosition(),
        new Rotation2d(e_turning.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(e_turning.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    e_driving.setPosition(0);
  }

  public double getDriveCurrent() {
    return m_driving.getOutputCurrent();
  }

  public double getTurnCurrent() {
    return m_turning.getOutputCurrent();
  }

  public double getVelocity(){
    return e_driving.getVelocity();
  }
}
