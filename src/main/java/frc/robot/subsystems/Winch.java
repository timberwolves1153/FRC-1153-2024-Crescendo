package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Winch extends SubsystemBase{
    
    private CANSparkMax leftMotor, rightMotor;
    private RelativeEncoder leftEncoder, rightEncoder;
    private SparkPIDController leftController, rightController;
    private final double leftUpSetpoint = -130;
    private final double rightUpSetpoint = 130;

    private final double leftDownSetpoint = -30;
    private final double rightDownSetpoint = 30;

    public Winch() {
        leftMotor = new CANSparkMax(60, MotorType.kBrushless);
        rightMotor = new CANSparkMax(61, MotorType.kBrushless);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        leftController = leftMotor.getPIDController();
        rightController = rightMotor.getPIDController();

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftController.setP(0.01);
        rightController.setP(0.01);
        leftController.setI(0.0);
        rightController.setI(0.0);
        leftController.setD(0.0);
        rightController.setD(0.0);
    

        leftMotor.clearFaults();
        rightMotor.clearFaults();
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
    
        leftMotor.setSmartCurrentLimit(40);
        rightMotor.setSmartCurrentLimit(40);

        // leftMotor.setInverted(false);
        
        //rightMotor.follow(leftMotor, true);
         
        
        
        leftMotor.burnFlash();
        rightMotor.burnFlash();
    }

    public void winchUp() {
        leftMotor.setVoltage(-11);
        rightMotor.setVoltage(11);
    }

    public void winchDown() {
        leftMotor.setVoltage(11);
        rightMotor.setVoltage(-11);
    }

    public void leftWinchDown() {
        leftMotor.setVoltage(11);
        
    }

    public void leftWinchUp() {
        leftMotor.setVoltage(-11);
    }

    public void leftStop() {
        leftMotor.setVoltage(0);
    }

    public void rightWinchUp() {
        rightMotor.setVoltage(8);
    }

    public void rightWinchDown() {
        rightMotor.setVoltage(-11);
    }

    public void rightStop() {
        rightMotor.setVoltage(0);
    }

    public void stop() {
        leftMotor.setVoltage(0);
        rightMotor.setVoltage(0);
    }

    public void pidWinchUp() {
        leftController.setReference(leftUpSetpoint, ControlType.kPosition);
        rightController.setReference(rightUpSetpoint, ControlType.kPosition);
    }

    public void pidWinchDown() {
        leftController.setReference(leftDownSetpoint, ControlType.kPosition);
        rightController.setReference(rightDownSetpoint, ControlType.kPosition);
    }

    public void resetEncoder() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }


    @Override
    public void periodic() {
        // SmartDashboard.putNumber("left climb encoder", leftEncoder.getPosition());
        // SmartDashboard.putNumber("right climb encoder", rightEncoder.getPosition());
    }

}
