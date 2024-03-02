package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final SparkPIDController kShootFollowPID = kShootFollow.getPIDController();
  private double targetSetPoint = 0;

  private final DigitalInput intakeProx = new DigitalInput(ShooterConstants.kIntakeProxDIO);
  private final DigitalInput loadedProx = new DigitalInput(ShooterConstants.kLoadedProxDIO);

  public ShooterSubsystem() {
    CommandScheduler.getInstance().registerSubsystem(this);

    kMidMain.setIdleMode(IdleMode.kBrake);
    kMidFollow.setIdleMode(IdleMode.kBrake);

    kIntake.setInverted(true);
    kMidFollow.follow(kMidMain, true);
    kMidMain.setInverted(false);
    //kShootFollow.follow(kShootMain, true);
    kShootMain.setInverted(true);
    kShootFollow.setInverted(false);

    kShootPID.setP(ShooterConstants.kShooterP);
    kShootPID.setI(ShooterConstants.kShooterI);
    kShootPID.setD(ShooterConstants.kShooterD);
    kShootPID.setFF(ShooterConstants.kShooterFF);
    kShootPID.setOutputRange(ShooterConstants.kShooterMinOutput, ShooterConstants.kShooterMaxOutput);

    kShootFollowPID.setP(ShooterConstants.kShooterP);
    kShootFollowPID.setI(ShooterConstants.kShooterI);
    kShootFollowPID.setD(ShooterConstants.kShooterD);
    kShootFollowPID.setFF(ShooterConstants.kShooterFF);
    kShootFollowPID.setOutputRange(ShooterConstants.kShooterMinOutput, ShooterConstants.kShooterMaxOutput);
  }

  public void setIntakeRollers(double setPoint) {
    kIntake.set(setPoint);
  }

  public void setMidRollers(double setPoint) {
    kMidMain.set(setPoint);
  }

  public void setShooterVelocity(double setPoint) {
    kShootPID.setReference(setPoint, ControlType.kVelocity);
    kShootFollowPID.setReference(setPoint, ControlType.kVelocity);
    targetSetPoint = setPoint;
  }

  public boolean atSpeed() {
    return Math.abs(kShootMain.getEncoder().getVelocity() - targetSetPoint) < ShooterConstants.kShooterSpeedDeadband;
  }

  public void intake() {
    setIntakeRollers(ShooterConstants.kIntakeSpeed);
    setMidRollers(ShooterConstants.kMidRollerIntakeSpeed);
  }

  public void stopRollers(boolean stopShooterRollers) {
    setIntakeRollers(0);
    setMidRollers(0);
    if(stopShooterRollers) {
      stopShooterRollers();
    }
  }

  public void stopShooterRollers() {
    kShootPID.setReference(0, ControlType.kDutyCycle);
    kShootFollowPID.setReference(0, ControlType.kDutyCycle);
    targetSetPoint = 0;
  }

  public void speedUp(double targetSpeed) {
    setMidRollers(ShooterConstants.kMidRollerGrabSpeed);
    setShooterVelocity(targetSpeed);
  }

  public void kickNote(boolean checkForSpeed) {
    if(targetSetPoint > 0 && checkForSpeed) {
      if(atSpeed()) {
        setMidRollers(ShooterConstants.kMidRollerKickSpeed);
      }
    }
    else {
      setMidRollers(ShooterConstants.kMidRollerKickSpeed);
    }
  }

  public void chainShoot(double targetSpeed) {
    setShooterVelocity(targetSpeed);
    intake();
  }

  public boolean hasNote() {
    return loadedProx.get();
  }

  public boolean intakingNote() {
    return intakeProx.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooterspeed", kShootMain.getEncoder().getVelocity());
  }

  public void tempSetSpeed(double speed) {
    kShootPID.setReference(speed, ControlType.kDutyCycle);
    kShootFollowPID.setReference(speed, ControlType.kDutyCycle);
  }
}
