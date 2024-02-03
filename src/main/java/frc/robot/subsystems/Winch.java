package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Winch extends SubsystemBase{
    
    private CANSparkMax leftMotor, rightMotor;


    public Winch() {
        leftMotor = new CANSparkMax(61, MotorType.kBrushless);
        rightMotor = new CANSparkMax(60, MotorType.kBrushless);
        
        
      // rightMotor.follow(leftMotor, false);

        leftMotor.restoreFactoryDefaults();
        leftMotor.clearFaults();
        leftMotor.setIdleMode(IdleMode.kBrake);
       // leftMotor.setInverted(false);
        leftMotor.setSmartCurrentLimit(40);

        rightMotor.clearFaults();
        rightMotor.setSmartCurrentLimit(40);
        rightMotor.restoreFactoryDefaults();
        rightMotor.setIdleMode(IdleMode.kBrake);

        
        leftMotor.burnFlash();
        rightMotor.burnFlash();
    }

    public void moveUp() {
        // sets both left and right motors bc follow()
        leftMotor.setVoltage(-6);
        rightMotor.setVoltage(6);
    }

    public void moveDown() {
        // sets both left and right motors bc follow()
        leftMotor.setVoltage(6);
        rightMotor.setVoltage(-6);
    }

    public void stop() {
        leftMotor.setVoltage(0);
        rightMotor.setVoltage(0);
    }

}
