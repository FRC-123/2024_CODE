package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax kIntake = new CANSparkMax(ShooterConstants.kIntakeCanId, MotorType.kBrushed);
  private final CANSparkMax kMidMain = new CANSparkMax(ShooterConstants.kMidMainCanId, MotorType.kBrushless);
  private final CANSparkMax kMidFollow = new CANSparkMax(ShooterConstants.kMidFollowCanId, MotorType.kBrushless);
  private final CANSparkMax kShootMain = new CANSparkMax(ShooterConstants.kShootMainCanId, MotorType.kBrushless);
  private final CANSparkMax kShootFollow = new CANSparkMax(ShooterConstants.kShootFollowCanId, MotorType.kBrushless);

  private final SparkPIDController kShootPID = kShootMain.getPIDController();
  private double targetSetPoint = 0;

  private final DigitalInput kIntakeProx = new DigitalInput(ShooterConstants.kIntakeProxDIO);
  private final DigitalInput kLoadedProx = new DigitalInput(ShooterConstants.kLoadedProxDIO);

  public ShooterSubsystem() {
    CommandScheduler.getInstance().registerSubsystem(this);

    kMidMain.setIdleMode(IdleMode.kBrake);
    kMidFollow.setIdleMode(IdleMode.kBrake);

    kIntake.setInverted(true);
    kMidFollow.follow(kMidMain, true);
    kMidMain.setInverted(false);
    kShootFollow.follow(kShootMain, true);
    kShootMain.setInverted(true);

    kShootPID.setP(ShooterConstants.kShooterP);
    kShootPID.setI(ShooterConstants.kShooterI);
    kShootPID.setD(ShooterConstants.kShooterD);
    kShootPID.setFF(ShooterConstants.kShooterFF);
    kShootPID.setOutputRange(ShooterConstants.kShooterMinOutput, ShooterConstants.kShooterMaxOutput);
  }

  public void setIntake(double setPoint) {
    kIntake.set(setPoint);
  }

  public void setMidRollers(double setPoint) {
    kMidMain.set(setPoint);
  }

  public void setShooterVelocity(double setPoint) {
    kShootPID.setReference(setPoint, ControlType.kVelocity);
    targetSetPoint = setPoint;
  }

  public boolean atSpeed() {
    return Math.abs(kShootMain.getEncoder().getVelocity() - targetSetPoint) < ShooterConstants.kShooterDeadband;
  }
}
