package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{
    private ShooterSubsystem m_ShooterSubsystem;

    public ShootCommand(ShooterSubsystem shooterSubsystem) {
        m_ShooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        m_ShooterSubsystem.speedUp(ShooterConstants.kShooterSpeedNormal);
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
