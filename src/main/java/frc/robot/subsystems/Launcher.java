package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase{
    
    private CANSparkMax m_Roller1, m_Roller2, m_leftPivot, m_rightPivot;
    private SparkPIDController roller1PID, pivotPID;
    private DutyCycleEncoder pivotEncoder;

    

    public Launcher() {
        m_leftPivot = new CANSparkMax(51, MotorType.kBrushless);
        m_rightPivot = new CANSparkMax(52, MotorType.kBrushless);
        m_Roller1 = new CANSparkMax(53, MotorType.kBrushless);
        m_Roller2 = new CANSparkMax(54, MotorType.kBrushless);

        

        roller1PID = m_Roller1.getPIDController();
        pivotPID = m_leftPivot.getPIDController();

        
        pivotEncoder = new DutyCycleEncoder(0);

        configMotors();
    }

    public void launchWithVolts() {
        m_Roller1.setVoltage(6);
       
    }

    public void stopLauncher() {
        m_Roller1.setVoltage(0);
        
    }

    public void pivotUp() {
        m_leftPivot.setVoltage(6);
       
    }

    public void pivotDown() {
        m_leftPivot.setVoltage(-6);
           }

    public void pivotStop() {
        m_leftPivot.setVoltage(0);
        
    }

    public void setLauncherVelocity(double setpoint) {
        roller1PID.setReference(setpoint, ControlType.kVelocity);
        
    }

    public double getAbsoluteMeasurement() {
        return pivotEncoder.getAbsolutePosition();
    }


    public void configMotors() {
        m_leftPivot.restoreFactoryDefaults();
        m_leftPivot.clearFaults();
        m_leftPivot.setIdleMode(IdleMode.kBrake);
        

        m_rightPivot.restoreFactoryDefaults();
        m_rightPivot.clearFaults();
        m_rightPivot.setIdleMode(IdleMode.kBrake);
        m_rightPivot.setInverted(true);
        m_rightPivot.follow(m_leftPivot);
        m_rightPivot.burnFlash();
        m_leftPivot.burnFlash();
        

        m_Roller1.restoreFactoryDefaults();
        m_Roller1.clearFaults();
        m_Roller1.setIdleMode(IdleMode.kCoast);

        m_Roller2.restoreFactoryDefaults();
        m_Roller2.clearFaults();
        m_Roller2.setIdleMode(IdleMode.kCoast);
        m_Roller2.setInverted(true);

        m_Roller2.follow(m_Roller1);
        m_Roller2.burnFlash();
        m_Roller1.burnFlash();
        
        roller1PID.setP(0.01);
        roller1PID.setI(0);
        roller1PID.setD(0); 

        // roller2PID.setI(0.01);
        // roller2PID.setI(0);
        // roller2PID.setI(0);

    }
}
