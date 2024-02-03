package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase{
    
    private CANSparkMax m_Roller1, m_Roller2, m_leftPivot, m_rightPivot;
    private SparkPIDController roller1PID, pivotPID;
    private DutyCycleEncoder pivotAbsoluteEncoder;

    private PIDController rollerController, pivotController;
    private SimpleMotorFeedforward rollerFF;
    private ArmFeedforward pivotFF;
    

    public Launcher() {
        m_leftPivot = new CANSparkMax(51, MotorType.kBrushless);
        m_rightPivot = new CANSparkMax(52, MotorType.kBrushless);
        m_Roller1 = new CANSparkMax(53, MotorType.kBrushless);
        m_Roller2 = new CANSparkMax(54, MotorType.kBrushless);

        // NEED TO TUNE
        pivotController = new PIDController(0, 0, 0);
        pivotFF = new ArmFeedforward(0, 0, 0);
        // NEED TO TUNE
        rollerController = new PIDController(0, 0, 0);
        rollerFF = new SimpleMotorFeedforward(0, 0, 0);

        
        pivotAbsoluteEncoder = new DutyCycleEncoder(0);

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

    public double getVelocity() {
        return m_Roller1.getEncoder().getVelocity();
    }

    public void setLauncherVelocity(double setpoint) {
        double feedback = rollerController.calculate(getVelocity(), setpoint);
        double feedforward = rollerFF.calculate(setpoint);
        m_Roller1.setVoltage(feedback + feedforward);
        
    }

    public double getAbsoluteMeasurement() {
        return pivotAbsoluteEncoder.getAbsolutePosition();
    }
    public double getPivotRadians() {
        return getAbsoluteMeasurement() * 2 * Math.PI;
    }

    public double getPivotDegrees() {
        return Math.toDegrees(getPivotRadians());
    }

    public void setPivotPosition(double degrees) {
        double setpointRads = Math.toRadians(degrees);
        double feedback = pivotController.calculate(getPivotRadians(), setpointRads);
        double feedforward = pivotFF.calculate(setpointRads, 1);
        m_leftPivot.setVoltage(feedback + feedforward);
    }

    @Override
    public void periodic() {
        if (Constants.launcherPivotTuningMode) {
            SmartDashboard.putNumber("pivot degrees", getPivotDegrees());
            SmartDashboard.putNumber("pivot absolute", getAbsoluteMeasurement());
            SmartDashboard.putNumber("pivot radians", getPivotRadians());
            SmartDashboard.putNumber("pivot setpoint", pivotController.getSetpoint());

        }
        if (Constants.launcherRollerTuningMode) {
            SmartDashboard.putNumber("roller velocity", getVelocity());
            SmartDashboard.putNumber("velocity setpoint", rollerController.getSetpoint());
        }
    }


    public void configMotors() {
        m_leftPivot.restoreFactoryDefaults();
        m_leftPivot.clearFaults();
        m_leftPivot.setIdleMode(IdleMode.kBrake);
        m_leftPivot.setInverted(false);

        m_rightPivot.restoreFactoryDefaults();
        m_rightPivot.clearFaults();
        m_rightPivot.setIdleMode(IdleMode.kBrake);
        m_rightPivot.follow(m_leftPivot, true);
        

        m_Roller1.restoreFactoryDefaults();
        m_Roller1.clearFaults();
        m_Roller1.setIdleMode(IdleMode.kCoast);
        m_Roller1.setInverted(false);

        m_Roller2.restoreFactoryDefaults();
        m_Roller2.clearFaults();
        m_Roller2.setIdleMode(IdleMode.kCoast);
        m_Roller2.follow(m_Roller1, true);

        m_Roller2.burnFlash();
        m_Roller1.burnFlash();
        m_rightPivot.burnFlash();
        m_leftPivot.burnFlash();
    }
}
