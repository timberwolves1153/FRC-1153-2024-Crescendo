package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Mailbox extends SubsystemBase {

    private CANSparkMax indexMotor;
    private double mailboxVolts;

    public DigitalInput bannerSensor;

    public Mailbox() {
        indexMotor = new CANSparkMax(55, MotorType.kBrushless);
        config();

        mailboxVolts = 0;

        bannerSensor = new DigitalInput(1);
       // SmartDashboard.putNumber("Mailbox Volts", mailboxVolts);
    }

    public void sendToLauncher() {
        indexMotor.setVoltage(12);
    }

    public void sendToIntake() {
        indexMotor.setVoltage(-12);
    }

    public void intake() {
        indexMotor.setVoltage(6);
    }

    public void stop() {
        indexMotor.setVoltage(0);
    }

    public void config() {
        indexMotor.restoreFactoryDefaults();
        indexMotor.setIdleMode(IdleMode.kBrake);
        indexMotor.clearFaults();
        indexMotor.setInverted(false);
        indexMotor.setSmartCurrentLimit(40);
        indexMotor.burnFlash();
    }

    public boolean getBannerSensor() {
        return !bannerSensor.get();
      }

    @Override
    public void periodic() {

        //  double mailboxV = SmartDashboard.getNumber("Mailbox Volts", mailboxVolts);
        // mailboxVolts = mailboxV;

        // if((mailboxVolts != mailboxV)) { 
        //     mailboxVolts = mailboxV;
        //  }

        SmartDashboard.putBoolean("Banner Sensor", getBannerSensor());

        
    }
}
