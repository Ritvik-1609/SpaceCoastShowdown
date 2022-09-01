package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCMD extends CommandBase {

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_drive;
    private final DoubleSupplier xSpeed;
    private final DoubleSupplier rot;

    public DriveCMD(DriveSubsystem subsystem, DoubleSupplier xSpeed, DoubleSupplier rot) {
        m_drive = subsystem;
        this.xSpeed = xSpeed;
        this.rot = rot;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    public void initialize() {

    }

    public void execute() {
        m_drive.drive(xSpeed.getAsDouble() * 3, rot.getAsDouble() * 3);
    }

    public void end() {
        m_drive.drive(0, 0);
    }

    public boolean isFinished() {
        return false;

    }

}
