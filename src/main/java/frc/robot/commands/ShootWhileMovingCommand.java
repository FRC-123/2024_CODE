package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWhileMovingCommand extends Command {
    private ShooterSubsystem m_ShooterSubsystem;
    private DriveSubsystem m_DriveSubsystem;

    public boolean atPlace = false;

    public ShootWhileMovingCommand(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem) {
        m_ShooterSubsystem = shooterSubsystem;
        m_DriveSubsystem = driveSubsystem;
        addRequirements(m_ShooterSubsystem);
    }

    @Override
    public void initialize() {
        m_ShooterSubsystem.speedUp(ShooterConstants.kShooterSpeedNormal + 750);
        //m_ShooterSubsystem.intake();
    }

    @Override
    public void execute() {
        if(!atPlace) {
            Transform2d diff = m_DriveSubsystem.getPose().minus(new Pose2d(1.3269, 5.553, new Rotation2d(0)));
            SmartDashboard.putString("diff", diff.toString());
            if(Math.abs(diff.getX()) < 0.3 && Math.abs(diff.getY()) < 1) {
                atPlace = true;
                m_ShooterSubsystem.kickNote(false);
                m_ShooterSubsystem.setIntakeRollers(ShooterConstants.kIntakeSpeed);
            }
        }
        else {
            if(!m_ShooterSubsystem.holdingNote) {
                m_ShooterSubsystem.setShooterVelocity(0);
                m_ShooterSubsystem.intake();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
