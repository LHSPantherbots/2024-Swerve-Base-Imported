package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final SparkMax m_IntakeFront;
    private final SparkMax m_IntakeBack;
    private final SparkMaxConfig c_IntakeFront;
    private final SparkMaxConfig c_IntakeBack;

    public Intake() {
        m_IntakeFront = new SparkMax(IntakeConstants.kFrontIntake, MotorType.kBrushless);
        m_IntakeBack = new SparkMax(IntakeConstants.kBackIntake, MotorType.kBrushless);
        c_IntakeFront = new SparkMaxConfig();
        c_IntakeBack = new SparkMaxConfig();

        c_IntakeFront
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30);

        c_IntakeBack
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30)
            .follow(m_IntakeFront);
        
        m_IntakeFront.configure(c_IntakeFront, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_IntakeBack.configure(c_IntakeBack, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {

    }

    public void intake() {
        m_IntakeFront.set(-.7);
    }

    public void outtake() {
        m_IntakeFront.set(.7);
    }

    public void intakeStop() {
        m_IntakeFront.set(0);
    }

    public double getCurrent() {
        return m_IntakeFront.getOutputCurrent();
    }

}
