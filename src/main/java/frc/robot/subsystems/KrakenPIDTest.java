package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KrakenPIDTest extends SubsystemBase{
    
    private final TalonFX m_left = new TalonFX(53);
    private final TalonFX m_right = new TalonFX(54);

    public KrakenPIDTest() {

        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        m_left.getConfigurator().apply(slot0Configs);
        m_right.getConfigurator().apply(slot0Configs);

        // create a velocity closed-loop request, voltage output, slot 0 configs


// set velocity to 8 rps, add 0.5 V to overcome gravity

    }


    public void runMotor() {
        final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
        m_left.setControl(m_request.withVelocity(70).withFeedForward(0.2));
    }

    public void stopMotor() {
        final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
        m_left.setControl(m_request.withVelocity(0).withFeedForward(0));
    }
}
