package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private XboxController m_drivController = new XboxController(OIConstants.kDriverControllerPort);

    public DefaultDriveCommand(DriveSubsystem subsystem) {
        this.driveSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if(m_drivController.getAButton()) {
            LimelightResults results = LimelightHelpers.getLatestResults("limelight");
            if(results.targetingResults.targets_Fiducials.length > 0 && (results.targetingResults.targets_Fiducials[0].fiducialID == 7 || results.targetingResults.targets_Fiducials[0].fiducialID == 4)) {
                double angle = Math.atan(results.targetingResults.targets_Fiducials[0].getTargetPose_CameraSpace().getX()/results.targetingResults.targets_Fiducials[0].getTargetPose_CameraSpace().getZ());
                SmartDashboard.putNumber("angle", angle);
                driveSubsystem.drive(0, 0, Math.pow(angle/-2.0, 11/5), false, true);
            }
            else {
                driveSubsystem.drive(0, 0, 0, false, true);
            }
        }
        int fineTurn = 0;
        if(m_drivController.getXButton()) {
            fineTurn += 1;
        }
        if(m_drivController.getBButton()) {
            fineTurn -= 1;
        }
        double multiplier = 0.4;
        double povmultiplier = 0.5;
        if((m_drivController.getLeftTriggerAxis() > 0.75) || (m_drivController.getRightTriggerAxis() > 0.75)) {
            multiplier = 0.65; //Turbo
            povmultiplier = 1; //POV Turbo
        }
        if(m_drivController.getPOV() == -1) {
            if(fineTurn == 0) {
                driveSubsystem.drive(
                    -multiplier*MathUtil.applyDeadband(m_drivController.getLeftY(), 0.015),
                    -multiplier*MathUtil.applyDeadband(m_drivController.getLeftX(), 0.015),
                    -multiplier*0.85*MathUtil.applyDeadband(m_drivController.getRightX(), 0.01),
                    true, true);
            }
            else {
                driveSubsystem.drive(
                    -multiplier*MathUtil.applyDeadband(m_drivController.getLeftY(), 0.015),
                    -multiplier*MathUtil.applyDeadband(m_drivController.getLeftX(), 0.015),
                    povmultiplier*fineTurn,
                    true, true);
            }
        }
        else {
            if(m_drivController.getPOV() == 0) {
                driveSubsystem.drive(povmultiplier*0.25, 0, povmultiplier*fineTurn, true, true);
            }
            else if(m_drivController.getPOV() == 45) {
                driveSubsystem.drive(povmultiplier*0.25, povmultiplier*-0.25, povmultiplier*fineTurn, true, true);
            }
            else if(m_drivController.getPOV() == 90) {
                driveSubsystem.drive(0, povmultiplier*-0.25, povmultiplier*fineTurn, true, true);
            }
            else if(m_drivController.getPOV() == 135) {
                driveSubsystem.drive(povmultiplier*-0.25, povmultiplier*-0.25, povmultiplier*fineTurn, true, true);
            }
            else if(m_drivController.getPOV() == 180) {
                driveSubsystem.drive(povmultiplier*-0.25, 0, povmultiplier*fineTurn, true, true);
            }
            else if(m_drivController.getPOV() == 225) {
                driveSubsystem.drive(povmultiplier*-0.25, povmultiplier*0.25, povmultiplier*fineTurn, true, true);
            }
            else if(m_drivController.getPOV() == 270) {
                driveSubsystem.drive(0, povmultiplier*0.25, povmultiplier*fineTurn, true, true);
            }
            else if(m_drivController.getPOV() == 315) {
                driveSubsystem.drive(povmultiplier*0.25, povmultiplier*0.25, povmultiplier*fineTurn, true, true);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
