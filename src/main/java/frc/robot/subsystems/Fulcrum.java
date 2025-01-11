package frc.robot.subsystems;
import java.util.Map;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.AbsoluteEncoder.Type;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants.FulcrumConstants;

public class Fulcrum extends SubsystemBase {
    private final SparkMax m_FulcrumRight;
    private final SparkMax m_FulcrumLeft;
    private final SparkMaxConfig c_FulcrumRight;
    private final SparkMaxConfig c_FulcrumLeft;

    private double lastSetpoint = 0;
    private double trimZero = 2.0;
    private double setPoint = trimZero;
    private double autoAimTrim = 0;
    

    //public double kP, kI, kD, kIz, kFF, kDt, kMaxOutput, kMinOutput, maxRPM, allowableError;      why was this called in the constructor??
    // PID coefficients these will need to be tuned
    public double kP = 0.02;// 0.00025; //5e-5;
    public double kI = 0;// 1e-6;
    public double kD = 0.0000;// 0.0004;
    public double kIz = 0;
    public double kFF = 0.0;// 0.00019;
    public double kDt = 0.02;
    public double kMaxOutput = 1;
    public double kMinOutput = -1;
    public double maxRPM = 5700;
    public double allowableError = 5; // 50 //Lets the system known when the velocity is close enough to launch
    private SparkClosedLoopController pidController;

    AbsoluteEncoder e_FulcrumEncoder;

    private final TrapezoidProfile.Constraints m_Constraints;
    private final ProfiledPIDController m_Controller;

    private ShuffleboardTab tab = Shuffleboard.getTab("Tuning");
    private GenericEntry sbAngle = tab.add("Fulcrum Angle", 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 90))
            .getEntry();

    static InterpolatingDoubleTreeMap kDistanceToFulcrumAngle = new InterpolatingDoubleTreeMap();

    static {
        kDistanceToFulcrumAngle.put(0.0, 9.0);
        kDistanceToFulcrumAngle.put(1.0, 9.0);
        kDistanceToFulcrumAngle.put(2.0, 20.0);
        // kDistanceToFulcrumAngle.put(2.25, 22.0);
        // kDistanceToFulcrumAngle.put(3.0, 28.0);
        // kDistanceToFulcrumAngle.put(3.5, 29.0);
        // kDistanceToFulcrumAngle.put(4.0, 32.5);
        kDistanceToFulcrumAngle.put(2.5, 25.0);
        kDistanceToFulcrumAngle.put(3.0, 27.0);
        kDistanceToFulcrumAngle.put(3.5, 30.0);
        kDistanceToFulcrumAngle.put(4.0, 33.0);
    }

    final DoubleSubscriber distanceSubscriber;

    public Fulcrum() {
        m_FulcrumRight = new SparkMax(FulcrumConstants.kFulcrumRight, MotorType.kBrushless);
        m_FulcrumLeft = new SparkMax(FulcrumConstants.kFulcrumLeft, MotorType.kBrushless);
        c_FulcrumRight = new SparkMaxConfig();
        c_FulcrumLeft = new SparkMaxConfig();
        e_FulcrumEncoder = m_FulcrumRight.getAbsoluteEncoder();
        pidController = m_FulcrumRight.getClosedLoopController();

        c_FulcrumRight
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        c_FulcrumRight.encoder
            .positionConversionFactor(360.0);
        c_FulcrumRight.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(kP, kI, kD,kFF)
            .maxOutput(kMaxOutput)
            .minOutput(kMinOutput); 

        c_FulcrumLeft
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .follow(m_FulcrumRight);
        c_FulcrumLeft.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(kP, kI, kD,kFF)
            .maxOutput(kMaxOutput)
            .minOutput(kMinOutput);   

        m_FulcrumRight.configure(c_FulcrumRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_FulcrumLeft.configure(c_FulcrumLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_Constraints = new TrapezoidProfile.Constraints(180, 180);
        m_Controller = new ProfiledPIDController(kP, kI, kD, m_Constraints, kDt);

        distanceSubscriber = NetworkTableInstance.getDefault().getDoubleTopic("/Distance").subscribe(0.0);

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Fulcrum is at Set Point", isAtPoint());
        SmartDashboard.putNumber("Fulcrum Velocity", e_FulcrumEncoder.getVelocity());
        SmartDashboard.putNumber("Fulcrum Output", m_FulcrumRight.getAppliedOutput());
        SmartDashboard.putNumber("Fulcrum Setpoint", setPoint);
        SmartDashboard.putNumber("Fulcrum Pos", e_FulcrumEncoder.getPosition());
        SmartDashboard.putBoolean("Fulcrum Down", isFulcurmDown());
        SmartDashboard.putNumber("Auto Angle", getAutoFulcrumAngle());
        SmartDashboard.putNumber("Fulcrum Trim",autoAimTrim);
    }

    public void manualFulcrum(double move) {
        // an if statment may need to be added to keep the fulcrum from going too far in
        // any given direction
        m_FulcrumRight.set(move);
    }

    public void manualFulcrumLeft(double move) {
        m_FulcrumLeft.set(move);
    }

    public void stopFulcrum() {
        m_FulcrumLeft.set(0.0);
        m_FulcrumRight.set(0.0);
    }

    public double getPosition() {
        return e_FulcrumEncoder.getPosition();
    }

    public void resetController() {
        m_Controller.reset(e_FulcrumEncoder.getPosition());
    }

    public boolean isAtPoint() {
        double error = getPosition() - setPoint;
        return (Math.abs(error) < allowableError);
    }

    public void stop() {
        m_FulcrumRight.set(0.0);
    }

    public double getSetPoint() {
        return setPoint;
    }

    public double getLastSetPoint() {
        return lastSetpoint;
    }

    public void setSetPoint(double point) {
        lastSetpoint = setPoint;
        setPoint = point;
    }

    public void setHorizontalHeight() {
        lastSetpoint = setPoint;
        // setPoint = tbd
        // closedLoopFulcrum(); this may or may not be needed
    }

    public void setAmpHeight() {
        lastSetpoint = setPoint;
        // setPoint = tbd
        // closedLoopFulcrum(); this may or may not be needed
    }

    public void setSpeakerHeight() {
        lastSetpoint = setPoint;
        // setPoint = tbd
        // closedLoopFulcrum(); this may or may not be needed
    }

    public void closedLoopFulcrum() {
        // m_FulcrumRight.set(m_Controller.calculate(e_FulcrumEncoder.getPosition(),
        // setPoint));
        pidController.setReference(setPoint, SparkMax.ControlType.kPosition);
    }

    public boolean isFulcurmDown() {
        return (e_FulcrumEncoder.getPosition() < 20.0);
    }

    // calculates estimated fulcrum angle to hit goal with out accounting for the
    // change in height as fulcrum moves
    public void autoAim() {
        // Double distance = SmartDashboard.getNumber("Distance To Target", .5);
        // Double theta = Math.toRadians(55.0); // angle of shooter relative to arm
        // Double Rz = 0.7; // Meters (Shooter Height)
        // Double Tz = 2.05; // Meters (Target Height)
        // Double alpha = Math.atan((Tz-Rz)/(distance)); // Radians (angle to target)
        // Double beta = theta - alpha; // radians (arm angle)
        lastSetpoint = setPoint;
        setPoint = getAutoFulcrumAngle();
        closedLoopFulcrum();
    }

    public double getSbAngle() {
        return sbAngle.getDouble(1.0);
    }

    public double getFulcrumAngleForDistance(double distance) {
        return kDistanceToFulcrumAngle.get(distance);
    }

    public double getAutoFulcrumAngle() {
        return getFulcrumAngleForDistance(distanceSubscriber.get()) + autoAimTrim;
    }

    public void resetTrim() {
        autoAimTrim = trimZero;
    }

    public void trimUp() {
        autoAimTrim += 0.5;
    }

    public void trimDown() {
        autoAimTrim -= 0.5;
    }

    public void setTrim(double trimValue){
        autoAimTrim = trimValue;
    }

}
