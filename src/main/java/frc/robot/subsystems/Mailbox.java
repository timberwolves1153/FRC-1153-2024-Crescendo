package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Mailbox extends SubsystemBase {

    private CANSparkMax indexMotor;

    public Mailbox() {
        indexMotor = new CANSparkMax(55, MotorType.kBrushless);
        config();
    }

    public void sendToLauncher() {
        indexMotor.setVoltage(6);
    }

    public void sendToIntake() {
        indexMotor.setVoltage(-6);
    }

    public void stop() {
        indexMotor.setVoltage(0);
    }

    public void config() {
        indexMotor.restoreFactoryDefaults();
        indexMotor.setIdleMode(IdleMode.kBrake);
        indexMotor.clearFaults();
        indexMotor.setInverted(false);
        indexMotor.burnFlash();
    }
}
