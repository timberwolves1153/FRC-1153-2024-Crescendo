package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BaseClef extends SubsystemBase{
    private CANSparkMax m_Clef;
    private RelativeEncoder encoder;
    private SparkPIDController controller;
    private final double setpoint = 30;

    public BaseClef() {
        m_Clef = new CANSparkMax(56, MotorType.kBrushless);
        encoder = m_Clef.getEncoder();
        controller = m_Clef.getPIDController();

        m_Clef.restoreFactoryDefaults();
        controller.setP(0.03);
        controller.setI(0);
        controller.setD(0.001);
        controller.setFF(0);
        m_Clef.setInverted(false);
        m_Clef.setIdleMode(IdleMode.kBrake);
        m_Clef.setSmartCurrentLimit(40);
        m_Clef.burnFlash();
    }

    public void manualDeploy() {
        m_Clef.setVoltage(3);
    }

    public void manualRetract() {
        m_Clef.setVoltage(-3);
    }

    public void stop() {
        m_Clef.setVoltage(0);
    }

    public void deployClef() {
        controller.setReference(setpoint, ControlType.kPosition);
    }

    public void retractClef() {
        controller.setReference(0, ControlType.kPosition);
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Clef Encoder", encoder.getPosition());
    }
}
