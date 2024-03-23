package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax kArmWinch = new CANSparkMax(ArmConstants.kArmWinchCanId, MotorType.kBrushed);
    //private final CANSparkMax kArmRoller = new CANSparkMax(ArmConstants.kArmRollerCanId, MotorType.kBrushed);
    //private final DigitalInput kEncoderSensor = new DigitalInput(ArmConstants.kEncoderDIO);

    private final XboxController controller = new XboxController(OIConstants.kArmControllerPort);

    //private int encoderCount = 0;
    //private boolean encoderState = false;
    //private ArmState targetState = ArmState.kDown;
    //private RollerState rollerState = RollerState.None;

    public ArmSubsystem() {
        CommandScheduler.getInstance().registerSubsystem(this);

        //kArmRoller.setIdleMode(IdleMode.kBrake);
        //kArmRoller.setInverted(true);
    }

    @Override
    public void periodic() {
        /*if(targetState.equals(ArmState.kUp)) {
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
        if(targetState.equals(ArmState.kUp) && encoderCount > 8) {
            kArmWinch.set(0);
        }
        SmartDashboard.putNumber("encoder", encoderCount);*/
        kArmWinch.set(controller.getRightY()*ArmConstants.kSpeed);
    }

    /*public void setState(ArmState state) {
        targetState = state;
        if(state == ArmState.kDown) {
            kArmWinch.set(ArmConstants.kDownSpeed);
        }
        else {
            if(encoderCount < 9) {
                kArmWinch.set(ArmConstants.kUpSpeed);
            }
        }
    }*/

    /*public void stopWinch() {
        kArmWinch.set(0);
    }*/

    /*public void intakeNote() {
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
    }*/

    /*public enum RollerState {
        Intake,
        Expell,
        None
    }*/
 }
// forward = down