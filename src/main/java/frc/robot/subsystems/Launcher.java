package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    
    private TalonFX m_leftLauncher, m_rightLauncher;
    private VoltageOut leftOutput, rightOutput;
    private VelocityVoltage m_leftRequest, m_rightRequest;
    private Slot0Configs slot0Configs;
    private TalonFXConfiguration leftMotorConfigs, rightMotorConfig;

    public Launcher() {

        m_leftLauncher = new TalonFX(53);
        m_rightLauncher = new TalonFX(54);

        leftMotorConfigs = new TalonFXConfiguration();
        rightMotorConfig = new TalonFXConfiguration();
        slot0Configs = new Slot0Configs();
        
        leftOutput = new VoltageOut(0);
        rightOutput = new VoltageOut(0);
        
        m_leftRequest = new VelocityVoltage(0).withSlot(0);
        m_rightRequest = new VelocityVoltage(0).withSlot(0);
        
        
        leftMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftMotorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        leftMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        rightMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        slot0Configs.kP = 0.01;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kS = 0; //tune
        slot0Configs.kV = 0.12; //from CTRE docs + Windham

        m_leftLauncher.getConfigurator().apply(slot0Configs);
        m_leftLauncher.getConfigurator().apply(leftMotorConfigs);
        m_rightLauncher.getConfigurator().apply(slot0Configs);
        m_rightLauncher.getConfigurator().apply(rightMotorConfig);
        
    //     config();
        
    }


    public void launchWithVolts() {
        m_leftLauncher.setControl(new VoltageOut(12));
        m_rightLauncher.setControl(new VoltageOut(7.5));
    }

    public void slowLaunchWithVolts() {
        m_leftLauncher.setControl(new VoltageOut(2.5));
        m_rightLauncher.setControl(new VoltageOut(2.5));
    }

    public void stopLaunchWithVolts() {
        m_leftLauncher.setControl(leftOutput.withOutput(0));
        m_rightLauncher.setControl(rightOutput.withOutput(0));
    }

    public void launchWithVelocity() {
        m_leftLauncher.setControl(m_leftRequest.withVelocity(4500));
        m_rightLauncher.setControl(m_rightRequest.withVelocity(3200));
    }

    public void stop() {
        m_leftLauncher.setControl(m_leftRequest.withVelocity(0));
        m_rightLauncher.setControl(m_rightRequest.withVelocity(0));
    }


    public void config() {


    }
}
