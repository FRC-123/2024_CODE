package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{
    private ShooterSubsystem m_ShooterSubsystem;
    private double speed;

    public ShootCommand(ShooterSubsystem shooterSubsystem, double speed) {
        m_ShooterSubsystem = shooterSubsystem;
        this.speed = speed;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        m_ShooterSubsystem.speedUp(speed);
    }

    @Override
    public void execute() {
        m_ShooterSubsystem.kickNote(true);
    }

    @Override
    public void end(boolean canceled) {
        m_ShooterSubsystem.stopRollers(true);
    }
}
