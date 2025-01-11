package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {

    private final SparkFlex m_LauncherTop;
    private final SparkFlex m_LauncherBottom;
    private final SparkFlexConfig c_LauncherTop;
    private final SparkFlexConfig c_LauncherBottom;
    private final RelativeEncoder e_LauncherTop;
    private final RelativeEncoder e_LauncherBottom;

    private double lastSetpoint = 0;
    private double setPoint = 0;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, allowableError;
    private SparkClosedLoopController pidController;

    static InterpolatingDoubleTreeMap kDistanceToShooterSpeed = new InterpolatingDoubleTreeMap();

    static {
        kDistanceToShooterSpeed.put(0.0, 5000.0);
        kDistanceToShooterSpeed.put(1.0, 5000.0);
        kDistanceToShooterSpeed.put(1.5, 5800.0);
        // kDistanceToShooterSpeed.put(2.0, 5000.0);
        // kDistanceToShooterSpeed.put(3.0, 5000.0);
        // kDistanceToShooterSpeed.put(4.0, 5225.0);
        kDistanceToShooterSpeed.put(2.0, 6200.0);
        kDistanceToShooterSpeed.put(3.0, 6600.0);
        kDistanceToShooterSpeed.put(4.0, 6800.0);
    }

    final DoubleSubscriber distanceSubscriber;

    public Launcher() {
        m_LauncherTop = new SparkFlex(LauncherConstants.kLauncherTop, MotorType.kBrushless);
        m_LauncherBottom = new SparkFlex(LauncherConstants.kLauncherBottom, MotorType.kBrushless);
        c_LauncherTop = new SparkFlexConfig();
        c_LauncherBottom = new SparkFlexConfig();

        c_LauncherTop
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60);
        c_LauncherTop.encoder
            .quadratureMeasurementPeriod(16);
        c_LauncherTop.absoluteEncoder
            .averageDepth(2);
        c_LauncherTop.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(kP, kI, kD,kFF)
            .maxOutput(kMaxOutput)
            .minOutput(kMinOutput); 

        c_LauncherBottom
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60)
            .follow(m_LauncherTop);
        c_LauncherBottom.encoder
            .quadratureMeasurementPeriod(16);
        c_LauncherBottom.absoluteEncoder
            .averageDepth(2);
        c_LauncherBottom.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(kP, kI, kD,kFF)
            .maxOutput(kMaxOutput)
            .minOutput(kMinOutput); 

        m_LauncherTop.configure(c_LauncherTop, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_LauncherBottom.configure(c_LauncherTop, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        e_LauncherTop = m_LauncherTop.getEncoder();
        e_LauncherBottom = m_LauncherTop.getEncoder();

        distanceSubscriber = NetworkTableInstance.getDefault().getDoubleTopic("/Distance").subscribe(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Current", m_LauncherTop.getOutputCurrent());
        SmartDashboard.putNumber("Launcer Top RPM", e_LauncherTop.getVelocity());
        SmartDashboard.putNumber("Launcer Bottom RPM", e_LauncherBottom.getVelocity());
        SmartDashboard.putNumber("Launcher SetPoint", setPoint);
        SmartDashboard.putBoolean("Launcher Is At Vel", isAtVelocity());
        SmartDashboard.putNumber("Auto RPM", getAutoShooterSpeed());
    }

    public void closedLoopLaunch() {
        pidController.setReference(setPoint, SparkMax.ControlType.kVelocity);
    }

    public void launcherRpmUp() {
        lastSetpoint = setPoint;
        setPoint = setPoint + 250;
        closedLoopLaunch();

    }

    public void launcherRpmDown() {
        lastSetpoint = setPoint;
        setPoint = setPoint - 250;
        closedLoopLaunch();
    }

    public void lancherMaxSpeed() {
        lastSetpoint = setPoint;
        setPoint = 6500;
        closedLoopLaunch();
    }

    public void lancherBloop(double velocity) {
        lastSetpoint = setPoint;
        setPoint = 4875 - 341.33 * velocity;
        closedLoopLaunch();
    }

    public void stopLauncher() {
        lastSetpoint = setPoint;
        setPoint = 0;
        closedLoopLaunch();
    }

    public void stopAll() {
        lastSetpoint = setPoint;
        setPoint = 0;
        closedLoopLaunch();
    }

    public void resumeLauncher() {
        var tmp = setPoint;
        setPoint = lastSetpoint;
        lastSetpoint = tmp;
        closedLoopLaunch();
    }

    public void setSetPoint(double point) {
        lastSetpoint = setPoint;
        this.setPoint = point;
        closedLoopLaunch();
    }

    public boolean isAtVelocity() {
        double error = e_LauncherTop.getVelocity() - setPoint;
        return (Math.abs(error) < allowableError);
    }

    public double getCurrent() {
        return m_LauncherTop.getOutputCurrent();
    }

    public double getShooterSpeedForDistance(double distance) {
        return kDistanceToShooterSpeed.get(distance);
    }

    public double getAutoShooterSpeed() {
        return getShooterSpeedForDistance(distanceSubscriber.get());
    }

    public void launcherAutoSpeed() {
        lastSetpoint = setPoint;
        setPoint = getAutoShooterSpeed();
        closedLoopLaunch();
    }

}
