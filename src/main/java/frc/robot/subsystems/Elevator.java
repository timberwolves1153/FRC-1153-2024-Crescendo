package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    
    private CANSparkMax leftMotor, rightMotor;


    public Elevator() {
        leftMotor = new CANSparkMax(61, MotorType.kBrushless);
        rightMotor = new CANSparkMax(60, MotorType.kBrushless);
        
        
      

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        leftMotor.clearFaults();
        rightMotor.clearFaults();
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
    
        leftMotor.setSmartCurrentLimit(40);
        rightMotor.setSmartCurrentLimit(40);

        // leftMotor.setInverted(false);
        
        rightMotor.follow(leftMotor, true);
         
        
        
        leftMotor.burnFlash();
        rightMotor.burnFlash();
    }

    public void moveUp() {
        // sets both left and right motors bc follow()
        leftMotor.setVoltage(-6);
        //rightMotor.setVoltage(6);
    }

    public void moveDown() {
        // sets both left and right motors bc follow()
        leftMotor.setVoltage(6);
        //rightMotor.setVoltage(-6);
    }

    public void stop() {
        leftMotor.setVoltage(0);
        //rightMotor.setVoltage(0);
    }

}
