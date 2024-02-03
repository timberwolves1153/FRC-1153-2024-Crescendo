package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase{
    
    private CANSparkMax m_leftRoller, m_rightRoller;

    private SparkPIDController leftRollerPID;
    private PIDController leftRollerController;
    
    private SimpleMotorFeedforward rollerFF;
    

    public Launcher() {
        m_leftRoller = new CANSparkMax(53, MotorType.kBrushless);
        m_rightRoller = new CANSparkMax(54, MotorType.kBrushless);

        // NEED TO TUNE
        leftRollerController = new PIDController(0, 0, 0);
        rollerFF = new SimpleMotorFeedforward(0, 0, 0);

        configMotors();
    }

    public void launchWithVolts() {
        m_leftRoller.setVoltage(6);
       
    }

    public void stopLauncher() {
        m_leftRoller.setVoltage(0);
        
    }

    public double getVelocity() {
        return m_leftRoller.getEncoder().getVelocity();
    }

    public void setLauncherVelocity(double setpoint) {
        double feedback = leftRollerController.calculate(getVelocity(), setpoint);
        double feedforward = rollerFF.calculate(setpoint);
        m_leftRoller.setVoltage(feedback + feedforward);
        
    }

    @Override
    public void periodic() {
        if (Constants.launcherRollerTuningMode) {
            SmartDashboard.putNumber("roller velocity", getVelocity());
            SmartDashboard.putNumber("velocity setpoint", leftRollerController.getSetpoint());
        }
    }


    public void configMotors() {
        m_leftRoller.restoreFactoryDefaults();
        m_leftRoller.clearFaults();
        m_leftRoller.setIdleMode(IdleMode.kCoast);
        m_leftRoller.setInverted(false);

        m_rightRoller.restoreFactoryDefaults();
        m_rightRoller.clearFaults();
        m_rightRoller.setIdleMode(IdleMode.kCoast);
        m_rightRoller.follow(m_leftRoller, true);

        m_leftRoller.burnFlash();
        m_rightRoller.burnFlash();
    }
}
