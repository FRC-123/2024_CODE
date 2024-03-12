package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WinchConstants;

public class WinchSubsystem extends SubsystemBase {
    private final CANSparkMax kWinch = new CANSparkMax(WinchConstants.kWinchCanId, MotorType.kBrushless);
    
    private final SparkPIDController kWinchPID = kWinch.getPIDController();

    private final XboxController controller = new XboxController(OIConstants.kArmControllerPort);

    public WinchSubsystem() {
        CommandScheduler.getInstance().registerSubsystem(this);

        kWinch.setIdleMode(IdleMode.kBrake);
        kWinch.getEncoder().setPosition(0);
        kWinch.setSoftLimit(SoftLimitDirection.kForward, 0);
        kWinch.setSoftLimit(SoftLimitDirection.kReverse, (float) WinchConstants.kTopPosition);
        kWinch.enableSoftLimit(SoftLimitDirection.kForward, true);
        kWinch.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    public void runWinch(double setPoint) {
        kWinch.set(setPoint);
    }

    @Override
    public void periodic() {
        runWinch(controller.getLeftY()*WinchConstants.kSpeed);
    }
    /*public void setWinchPosition(double setPoint) {
        kWinchPID.setReference(setPoint, ControlType.kPosition);
        targetPosition = setPoint;
    }

    public boolean atPosition() {
        return Math.abs(kWinch.getEncoder().getPosition() - targetPosition) < WinchConstants.kPositionDeadband;
    }*/
}
