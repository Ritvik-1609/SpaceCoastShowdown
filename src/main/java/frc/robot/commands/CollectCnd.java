package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CollectorSubsystem;

public class CollectCnd extends CommandBase {

    private final double speed;
    private final boolean inverted;
    private final boolean doubleBall;
    private CollectorSubsystem collector;

    public CollectCnd(double speed, CollectorSubsystem collector, boolean inverted, boolean doubleBall) {
        this.speed = speed;
        this.inverted = inverted;
        this.collector = collector;
        this.doubleBall = doubleBall;
        this.addRequirements(collector);
    }

    public void initialize() {
        collector.setMotor(0, false);
    }

    public void execute() {
        if (doubleBall) {
            collector.doubleBall();
        } else {
            collector.setMotor(speed, inverted);

        }
    }

    public void end() {
        collector.setMotor(0, false);
    }

    public boolean isFinished() {
        return false;
    }
}
