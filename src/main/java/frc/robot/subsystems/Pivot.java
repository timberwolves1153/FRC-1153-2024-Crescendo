package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase{
    
    private CANSparkMax m_leftPivot, m_rightPivot;
    private SparkPIDController pivotPID;
    private DutyCycleEncoder pivotAbsoluteEncoder;

    private PIDController pivotController;
    private ArmFeedforward pivotFF;
    

    public Pivot() {
        m_leftPivot = new CANSparkMax(51, MotorType.kBrushless);
        m_rightPivot = new CANSparkMax(52, MotorType.kBrushless);

        // NEED TO TUNE
        pivotController = new PIDController(0, 0, 0);
        pivotFF = new ArmFeedforward(0, 0, 0);
        
        pivotAbsoluteEncoder = new DutyCycleEncoder(0);

        configMotors();
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
        
        m_rightPivot.burnFlash();
        m_leftPivot.burnFlash();
    }
}
