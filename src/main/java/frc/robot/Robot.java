// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LedSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  /*private CANSparkMax intake = new CANSparkMax(ShooterConstants.kIntakeCanId, MotorType.kBrushed);
  private CANSparkMax midmain = new CANSparkMax(ShooterConstants.kMidMainCanId, MotorType.kBrushless);
  private CANSparkMax midfollow = new CANSparkMax(ShooterConstants.kMidFollowCanId, MotorType.kBrushless);
  private CANSparkMax shootmain = new CANSparkMax(ShooterConstants.kShootMainCanId, MotorType.kBrushless);
  private CANSparkMax shootfollow = new CANSparkMax(ShooterConstants.kShootFollowCanId, MotorType.kBrushless);*/

  private XboxController testController = new XboxController(1);
  private RobotContainer m_robotContainer;
  //private LedSubsystem m_led_subsystem; // we need an instance to run during init

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    LimelightHelpers.setPipelineIndex("limelight", 0);
    LimelightHelpers.setLEDMode_ForceOff("limelight");
    LimelightHelpers.getLatestResults("limelight");
    //CameraServer.startAutomaticCapture();
    /*midmain.setIdleMode(IdleMode.kBrake);
    midfollow.setIdleMode(IdleMode.kBrake);
    intake.setInverted(true);
    midfollow.follow(midmain, true);
    midmain.setInverted(false);
    shootfollow.follow(shootmain, true);
    shootmain.setInverted(true);*/
    //m_robotContainer.setBrakeMode(IdleMode.kCoast);
    //m_led_subsystem = new LedSubsystem();   // create an instance, which will initialize all buffers etc
    // addPeriodic(() -> our_alliance = DriverStation.getAlliance(), 1);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    if(LedSubsystem.dynamic) {
      LedSubsystem.set_dynamic_message();
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    LedSubsystem.set_blank_msg();
    //LimelightHelpers.setLEDMode_ForceOff("limelight");
    //m_robotContainer.setBrakeMode(IdleMode.kBrake);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    LedSubsystem.dynamic = true;
    //m_robotContainer.setBrakeMode(IdleMode.kCoast);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    //m_robotContainer.setBrakeMode(IdleMode.kCoast);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    LedSubsystem.set_our_alliance_solid();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(testController.getRightBumper()) {
      m_robotContainer.m_ShooterSubsystem.setShooterVelocity(SmartDashboard.getNumber("Shoot Speed", 4000));
    }
    else {
      m_robotContainer.m_ShooterSubsystem.stopShooterRollers();
    }
    /*double midspeed = 0;
    if(testController.getAButton()) {
      intake.set(0.6);
      midspeed = 0.4;
    }
    else {
      intake.set(0);
    }
    if(testController.getXButton()) {
      midspeed = -0.03;
      shootmain.set(SmartDashboard.getNumber("shoot", 0.75));
    }
    else {
      shootmain.set(0);
    }
    if(testController.getYButton()) {
      midspeed = 0.4;
    }
    midmain.set(midspeed);*/
    /*double intakespeed = 0;
    if(detect.get()) {
      midmain.set(-.05);
    }
    else {
      midmain.set(SmartDashboard.getNumber("mid", 0.14));
      if(testController.getAButton()) {
        intakespeed = SmartDashboard.getNumber("intake", 0.3);
      }
    }
    intake.set(intakespeed);
    if(testController.getXButton()) {
      midmain.set(0.2);
      shootmain.set(0.75);
    }
    else {
      shootmain.set(0);
    }*/
    //shootmain.set(SmartDashboard.getNumber("shoot", 0.1));
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    /*double midspeed = 0;
    if(testController.getAButton()) {
      intake.set(0.4);
      midspeed = 0.4;
    }
    else {
      intake.set(0);
    }
    if(testController.getXButton()) {
      midspeed = -0.03;
      shootmain.set(SmartDashboard.getNumber("shoot", 0.75));
    }
    else {
      shootmain.set(0);
    }
    if(testController.getYButton()) {
      midspeed = 0.4;
    }
    midmain.set(midspeed);*/
    /*double intakespeed = 0;
    if(detect.get()) {
      midmain.set(-.05);
    }
    else {
      midmain.set(SmartDashboard.getNumber("mid", 0.14));
      if(testController.getAButton()) {
        intakespeed = SmartDashboard.getNumber("intake", 0.3);
      }
    }
    intake.set(intakespeed);
    if(testController.getXButton()) {
      midmain.set(0.2);
      shootmain.set(0.75);
    }
    else {
      shootmain.set(0);
    }*/
    //shootmain.set(SmartDashboard.getNumber("shoot", 0.1));
  }
}
