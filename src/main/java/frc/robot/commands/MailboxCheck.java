package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Mailbox;

public class MailboxCheck extends Command{
    
    private Collector collector;
    private boolean bannerSensor;
    private Mailbox mailbox;

    public MailboxCheck(Collector collector, Mailbox mailbox){

        this.collector = collector;
        this.mailbox = mailbox;
    }

    @Override
    public void execute() {

        if (mailbox.getBannerSensor()) {
            mailbox.stop();
            
        } else {
            
            mailbox.intake();
        }
    }
    
    @Override
    public boolean isFinished() {
    if (mailbox.getBannerSensor()) {
        return true;
    } else {
        return false;
    }
}
}
