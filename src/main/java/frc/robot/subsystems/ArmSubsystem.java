package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax kArmWinch = new CANSparkMax(ArmConstants.kArmWinchCanId, MotorType.kBrushed);
    private final CANSparkMax kArmRoller = new CANSparkMax(ArmConstants.kArmRollerCanId, MotorType.kBrushed);
    private final DigitalInput kEncoderSensor = new DigitalInput(ArmConstants.kEncoderDIO);

    private int encoderCount = 0;
    private boolean encoderState = false;
    private ArmState targetState = ArmState.kDown;
    //private RollerState rollerState = RollerState.None;

    public ArmSubsystem() {
        CommandScheduler.getInstance().registerSubsystem(this);

        kArmRoller.setIdleMode(IdleMode.kBrake);
        kArmRoller.setInverted(true);
    }

    @Override
    public void periodic() {
        if(targetState.equals(ArmState.kUp)) {
            if(encoderState == false && kEncoderSensor.get()) {
                encoderCount++;
            }
        }
        else {
            if(encoderState == true && !kEncoderSensor.get()) {
                encoderCount--;
            }
        }
        encoderState = kEncoderSensor.get();
        if(kArmWinch.getForwardLimitSwitch(Type.kNormallyOpen).isPressed()) {
            encoderCount = 0;
        }
        if(targetState.equals(ArmState.kUp) && encoderCount > 7) {
            kArmWinch.set(0);
        }
        SmartDashboard.putNumber("encoder", encoderCount);
    }

    public void setState(ArmState state) {
        targetState = state;
        if(state == ArmState.kDown) {
            kArmWinch.set(ArmConstants.kDownSpeed);
        }
        else {
            if(encoderCount < 8) {
                kArmWinch.set(ArmConstants.kUpSpeed);
            }
        }
    }

    public void stopWinch() {
        kArmWinch.set(0);
    }

    public void intakeNote() {
        kArmRoller.set(ArmConstants.kIntakeSpeed);
    }

    public void expellNote() {
        kArmRoller.set(ArmConstants.kExpellSpeed);
    }

    public void stopRoller() {
        kArmRoller.set(ArmConstants.kHoldSpeed);
    }
    
    public enum ArmState {
        kDown,
        kUp
    }

    /*public enum RollerState {
        Intake,
        Expell,
        None
    }*/
 }
// forward = down