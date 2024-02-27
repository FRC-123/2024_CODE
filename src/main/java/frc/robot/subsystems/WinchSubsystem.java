package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WinchConstants;

public class WinchSubsystem extends SubsystemBase {
    private final CANSparkMax kWinch = new CANSparkMax(WinchConstants.kWinchCanId, MotorType.kBrushless);
    
    private final SparkPIDController kWinchPID = kWinch.getPIDController();
    private double targetPosition = 0;

    public WinchSubsystem() {
        CommandScheduler.getInstance().registerSubsystem(this);

        kWinch.setIdleMode(IdleMode.kBrake);
        kWinchPID.setP(WinchConstants.kWinchP);
        kWinchPID.setI(WinchConstants.kWinchI);
        kWinchPID.setD(WinchConstants.kWinchD);
        kWinchPID.setFF(WinchConstants.kWinchFF);
        kWinchPID.setOutputRange(WinchConstants.kWinchMinOutput, WinchConstants.kWinchMaxOutput);
    }

    public void setWinchPosition(double setPoint) {
        kWinchPID.setReference(setPoint, ControlType.kPosition);
        targetPosition = setPoint;
    }

    public boolean atPosition() {
        return Math.abs(kWinch.getEncoder().getPosition() - targetPosition) < WinchConstants.kPositionDeadband;
    }
}
