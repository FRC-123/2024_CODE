// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final WinchSubsystem m_WinchSubsystem = new WinchSubsystem();
  //private final HiArmSubsystem m_hiArm = new HiArmSubsystem();
  //private final LoArmSubsystem m_loArm = new LoArmSubsystem(m_hiArm);

  //private Trajectory test_traj;
  private boolean atPlace;

  Pose2d centerNotePoint = new Pose2d(2.896, 5.553, new Rotation2d(0));

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_armController = new XboxController(OIConstants.kArmControllerPort);
  CommandXboxController m_armControllerCommand = new CommandXboxController(OIConstants.kArmControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(new DefaultDriveCommand(m_robotDrive));
    //m_ShooterSubsystem.setDefaultCommand(new ShootWhileMovingCommand(m_ShooterSubsystem, m_robotDrive));
    SendableChooser<AutoType> autoType = new SendableChooser<AutoType>();
    autoType.addOption("One Piece", AutoType.One_Piece);
    autoType.addOption("Two Piece", AutoType.Two_Piece);
    autoType.addOption("Three Piece Right", AutoType.Three_Piece_Right);
    autoType.setDefaultOption("Three Piece Left", AutoType.Three_Piece_Left);
    SendableChooser<AutoAngle> autoAngle = new SendableChooser<AutoAngle>();
    autoAngle.addOption("Left", AutoAngle.Left);
    autoAngle.setDefaultOption("Right", AutoAngle.Right);
    SmartDashboard.putData("Auto Type", autoType);
    SmartDashboard.putData("Auto Angle", autoAngle);
    SmartDashboard.putNumber("Auto Delay", 0);
    SmartDashboard.putNumber("Aiming kp", 5.0);
    SmartDashboard.putNumber("Aiming minsteer", 0.035);
    SmartDashboard.putNumber("Aiming deadband", 0.05);
    //SmartDashboard.putNumber("limelight constant", 25);
    //SmartDashboard.putNumber("limelight kp", 0.15);
    /*try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("testcurve.wpilib.json");
        test_traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        test_traj = new Trajectory();
    }
    SmartDashboard.putString("start", test_traj.getInitialPose().toString());*/
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kY.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    
    new JoystickButton(m_driverController, Button.kStart.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    new Trigger(this::R1Up)
        .onTrue(new InstantCommand(() -> m_ArmSubsystem.setState(ArmState.kUp)))
        .onFalse(new InstantCommand(() -> m_ArmSubsystem.setState(ArmState.kDown)));
    new Trigger(this::R1Down)
        .onTrue(new InstantCommand(() -> m_ArmSubsystem.setState(ArmState.kDown)));

    m_armControllerCommand.povUp()
        .onTrue(new InstantCommand(() -> m_ArmSubsystem.expellNote()))
        .onFalse(new InstantCommand(() -> m_ArmSubsystem.stopRoller()));
    m_armControllerCommand.povDown()
        .onTrue(new InstantCommand(() -> m_ArmSubsystem.intakeNote()))
        .onFalse(new InstantCommand(() -> m_ArmSubsystem.stopRoller()));

    /*new Trigger(this::L1Up)
        .onTrue(new InstantCommand(() -> m_WinchSubsystem.setWinchPosition(300)))
        .onFalse(new InstantCommand(() -> m_WinchSubsystem.setWinchPosition(0)));
    new Trigger(this::L1Down)
        .onTrue(new InstantCommand(() -> m_WinchSubsystem.setWinchPosition(0)));*/
    
    m_armControllerCommand.x().onTrue(new InstantCommand(() -> LedSubsystem.set_top_load_req()));
    m_armControllerCommand.y().onTrue(new InstantCommand(() -> LedSubsystem.set_floor_req()));
    //m_armControllerCommand.rightBumper().onTrue(new InstantCommand(() -> LedSubsystem.set_our_alliance_solid()));
    m_armControllerCommand.rightBumper().whileTrue(new ShootCommand(m_ShooterSubsystem, 5000));
    m_armControllerCommand.leftBumper().onTrue(new InstantCommand(() -> LedSubsystem.dynamic = true));

    m_armControllerCommand.a().whileTrue(new StartEndCommand(() -> m_ShooterSubsystem.intake(), () -> m_ShooterSubsystem.stopRollers(false), m_ShooterSubsystem));
    new Trigger(this::rightTrigger).whileTrue(new ShootCommand(m_ShooterSubsystem, ShooterConstants.kShooterSpeedNormal));
    new Trigger(this::leftTrigger).whileTrue(new ShootCommand(m_ShooterSubsystem, 1400));
    m_armControllerCommand.b().whileTrue(new StartEndCommand(() -> m_ShooterSubsystem.setIntakeRollers(ShooterConstants.kIntakeUnjamSpeed), () -> m_ShooterSubsystem.stopRollers(false), m_ShooterSubsystem));
  }
    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_ShooterSubsystem.holdingNote = true;
    SendableChooser<AutoType> type = (SendableChooser<AutoType>) SmartDashboard.getData("Auto Type");
    SendableChooser<AutoAngle> angle = (SendableChooser<AutoAngle>) SmartDashboard.getData("Auto Angle");  //IMPORTANT: subwoofer angle is 120 degrees
    double delay = SmartDashboard.getNumber("Auto Delay", 0);
    //SendableChooser<AutoPiece> piece = (SendableChooser) SmartDashboard.getData("Auto Piece");
    //SendableChooser<AutoRotate> rotate = (SendableChooser) SmartDashboard.getData("Auto Rot");

    //return new InstantCommand();
    // Create a voltage constraint to ensure we don't accelerate too fast
    // var autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(
    //             DriveConstants.ksVolts,
    //             DriveConstants.kvVoltSecondsPerMeter,
    //             DriveConstants.kaVoltSecondsSquaredPerMeter),
    //         DriveConstants.kDriveKinematics,
    //         5);

    m_robotDrive.zeroHeading();
    if(type.getSelected().equals(AutoType.One_Piece) || type.getSelected().equals(AutoType.Angled_Two_Piece)) {
        if(angle.getSelected().equals(AutoAngle.Left)) {
            m_robotDrive.setFieldRelativeOffset(60);
        }
        else {
            m_robotDrive.setFieldRelativeOffset(-60);
        }
    }
    //m_robotDrive.resetOdometry(moveBackTraj.getInitialPose());

    //shootWhileCommand = new ShootWhileMovingCommand(m_ShooterSubsystem, m_robotDrive);
    //return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive);
    /*return new InstantCommand(() -> {m_ShooterSubsystem.tempSetSpeed(0.45);
            m_ShooterSubsystem.setMidRollers(ShooterConstants.kMidRollerGrabSpeed);
        }, m_ShooterSubsystem)
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(() -> m_ShooterSubsystem.kickNote(false), m_ShooterSubsystem))
        .andThen(new WaitCommand(0.75))
        .andThen(new ParallelCommandGroup(swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive), new InstantCommand(() -> {
            m_ShooterSubsystem.stopShooterRollers();
            m_ShooterSubsystem.intake();
        })))
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(() -> {
            m_ShooterSubsystem.stopRollers(false);
            m_ShooterSubsystem.tempSetSpeed(0.45);
            m_ShooterSubsystem.setMidRollers(ShooterConstants.kMidRollerGrabSpeed);
        }))
        .andThen(new InstantCommand(() -> m_robotDrive.resetOdometry(exampleTrajectoryBack.getInitialPose())))
        .andThen(new RunCommand(() -> m_robotDrive.drive(-0.3, 0, 0, false, true), m_robotDrive)
                    .until(() -> m_robotDrive.getPose().minus(new Pose2d(-1, 0, new Rotation2d(0))).getX() <= 0))
        .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive)
        .andThen(() -> m_ShooterSubsystem.kickNote(false))
        .andThen(new WaitCommand(1))
        .andThen(() -> m_ShooterSubsystem.stopRollers(true));*/
        
    if(type.getSelected().equals(AutoType.Three_Piece_Left)) { // Doesn't use auto angle, might use alliance color
        return new InstantCommand(() -> m_ShooterSubsystem.speedUp(ShooterConstants.kShooterSpeedNormal))
            .andThen(new WaitUntilCommand(m_ShooterSubsystem::atSpeed))
            .andThen(() -> m_ShooterSubsystem.kickNote(false))
            .andThen(new WaitUntilCommand(() -> !m_ShooterSubsystem.holdingNote))
            .andThen(new WaitCommand(0.25))
            .andThen(() -> m_ShooterSubsystem.stopRollers(true))
            .andThen(() -> m_ShooterSubsystem.setShooterVelocity(0))
            .andThen(() -> m_ShooterSubsystem.intake())
            .andThen(backUpCommand(true))
            .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive)
            .andThen(() -> {
                //m_ShooterSubsystem.stopRollers(false);
                //m_ShooterSubsystem.setMidRollers(ShooterConstants.kMidRollerGrabSpeed);
                atPlace = false;
                })
            //.andThen(new WaitCommand(0.5))
            .andThen(new ParallelCommandGroup(thirdNotePathLeft(), new WaitCommand(0.5)
                .andThen(() -> {
                    m_ShooterSubsystem.stopRollers(false);
                    m_ShooterSubsystem.setMidRollers(ShooterConstants.kMidRollerGrabSpeed);
                })
                .andThen(new WaitCommand(0.25))
                .andThen(() -> m_ShooterSubsystem.speedUp(2500))
                .andThen(new WaitUntilCommand(this::atPlace))
                .andThen(() -> {
                    m_ShooterSubsystem.kickNote(false);
                    m_ShooterSubsystem.setIntakeRollers(ShooterConstants.kIntakeSpeed);
                })
                .andThen(new WaitCommand(0.25))
                .andThen(() -> m_ShooterSubsystem.setShooterVelocity(0))
                .andThen(() -> m_ShooterSubsystem.intake())
            ))
            //.andThen(new ParallelCommandGroup(thirdNotePath(), new WaitCommand(0.25).andThen(() -> m_ShooterSubsystem.speedUp(2500)).andThen(shootWhileCommand).andThen(new WaitCommand(0.25)).andThen(() -> m_ShooterSubsystem.setShooterVelocity(0)).andThen(() -> m_ShooterSubsystem.intake())))
            .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive)
            .andThen(new ParallelCommandGroup(fourthNothPathLeft(), new WaitCommand(0.5)
                .andThen(() -> {
                    m_ShooterSubsystem.stopRollers(false);
                    m_ShooterSubsystem.setMidRollers(ShooterConstants.kMidRollerGrabSpeed);/*g */
                })
                .andThen(new WaitCommand(0.25))
                .andThen(() -> m_ShooterSubsystem.speedUp(ShooterConstants.kShooterSpeedNormal + 250))
            ))
            .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive)
            .andThen(() -> m_ShooterSubsystem.kickNote(false))
            .andThen(new WaitCommand(0.25))
            .andThen(() -> m_ShooterSubsystem.stopRollers(true));
    }
    else if(type.getSelected().equals(AutoType.Three_Piece_Right)) {
        return new InstantCommand(() -> m_ShooterSubsystem.speedUp(ShooterConstants.kShooterSpeedNormal))
            .andThen(new WaitUntilCommand(m_ShooterSubsystem::atSpeed))
            .andThen(() -> m_ShooterSubsystem.kickNote(false))
            .andThen(new WaitUntilCommand(() -> !m_ShooterSubsystem.holdingNote))
            .andThen(new WaitCommand(0.25))
            .andThen(() -> m_ShooterSubsystem.stopRollers(true))
            .andThen(() -> m_ShooterSubsystem.setShooterVelocity(0))
            .andThen(() -> m_ShooterSubsystem.intake())
            .andThen(backUpCommand(true))
            .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive)
            .andThen(() -> {
                //m_ShooterSubsystem.stopRollers(false);
                //m_ShooterSubsystem.setMidRollers(ShooterConstants.kMidRollerGrabSpeed);
                atPlace = false;
                })
            //.andThen(new WaitCommand(0.5))
            .andThen(new ParallelCommandGroup(thirdNotePathRight(), new WaitCommand(0.5)
                .andThen(() -> {
                    m_ShooterSubsystem.stopRollers(false);
                    m_ShooterSubsystem.setMidRollers(ShooterConstants.kMidRollerGrabSpeed);
                })
                .andThen(new WaitCommand(0.25))
                .andThen(() -> m_ShooterSubsystem.speedUp(2500))
                .andThen(new WaitUntilCommand(this::atPlace))
                .andThen(() -> {
                    m_ShooterSubsystem.kickNote(false);
                    m_ShooterSubsystem.setIntakeRollers(ShooterConstants.kIntakeSpeed);
                })
                .andThen(new WaitCommand(0.25))
                .andThen(() -> m_ShooterSubsystem.setShooterVelocity(0))
                .andThen(() -> m_ShooterSubsystem.intake())
            ))
            //.andThen(new ParallelCommandGroup(thirdNotePath(), new WaitCommand(0.25).andThen(() -> m_ShooterSubsystem.speedUp(2500)).andThen(shootWhileCommand).andThen(new WaitCommand(0.25)).andThen(() -> m_ShooterSubsystem.setShooterVelocity(0)).andThen(() -> m_ShooterSubsystem.intake())))
            .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive)
            .andThen(new ParallelCommandGroup(fourthNothPathRight(), new WaitCommand(0.5)
                .andThen(() -> {
                    m_ShooterSubsystem.stopRollers(false);
                    m_ShooterSubsystem.setMidRollers(ShooterConstants.kMidRollerGrabSpeed);/*g */
                })
                .andThen(new WaitCommand(0.25))
                .andThen(() -> m_ShooterSubsystem.speedUp(ShooterConstants.kShooterSpeedNormal + 250))
            ))
            .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive)
            .andThen(() -> m_ShooterSubsystem.kickNote(false))
            .andThen(new WaitCommand(0.25))
            .andThen(() -> m_ShooterSubsystem.stopRollers(true));
    }
    else if(type.getSelected().equals(AutoType.Two_Piece)) { // Doesn't use auto angle or color
        return new InstantCommand(() -> m_ShooterSubsystem.speedUp(ShooterConstants.kShooterSpeedNormal))
            .andThen(new WaitUntilCommand(m_ShooterSubsystem::atSpeed))
            .andThen(() -> m_ShooterSubsystem.kickNote(false))
            .andThen(new WaitUntilCommand(() -> !m_ShooterSubsystem.holdingNote))
            .andThen(new WaitCommand(0.25))
            .andThen(() -> m_ShooterSubsystem.stopRollers(true))
            .andThen(() -> m_ShooterSubsystem.setShooterVelocity(0))
            .andThen(() -> m_ShooterSubsystem.intake())
            .andThen(backUpCommand(true))
            .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive)
            .andThen(() -> {
                m_ShooterSubsystem.stopRollers(false);
                m_ShooterSubsystem.setMidRollers(ShooterConstants.kMidRollerGrabSpeed);
                })
            .andThen(new ParallelCommandGroup(moveToSpeakerCommand(), new WaitCommand(0.25).andThen(() -> m_ShooterSubsystem.speedUp(2500))))
            .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive)
            .andThen(() -> m_ShooterSubsystem.kickNote(false))
            .andThen(new WaitCommand(0.5))
            .andThen(() -> m_ShooterSubsystem.stopRollers(true));
    }
    else if(type.getSelected().equals(AutoType.Four_Piece)) { //Doesn't use angle, might use color
        return new InstantCommand();
    }
    else if(type.getSelected().equals(AutoType.Angled_Two_Piece)) { //Uses angle, might use color
        return new InstantCommand();
    }
    else { //Uses angle, uses color
        return new WaitCommand(delay)
            .andThen(() -> m_ShooterSubsystem.speedUp(ShooterConstants.kShooterSpeedNormal))
            .andThen(new WaitUntilCommand(m_ShooterSubsystem::atSpeed))
            .andThen(() -> m_ShooterSubsystem.kickNote(false))
            .andThen(new WaitUntilCommand(() -> !m_ShooterSubsystem.holdingNote))
            .andThen(new WaitCommand(0.25))
            .andThen(() -> m_ShooterSubsystem.stopRollers(true))
            .andThen(getOutCommand(true, angle.getSelected(), DriverStation.getAlliance().get()))
            .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive);
    }
    /*return new InstantCommand(() -> m_ShooterSubsystem.speedUp(ShooterConstants.kShooterSpeedNormal))
        .andThen(new WaitCommand(0.5))
        .andThen(new InstantCommand(() -> m_ShooterSubsystem.kickNote(false)))
        .andThen(new ParallelCommandGroup(moveBack.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive), 
            new WaitUntilCommand(() -> !m_ShooterSubsystem.hasNote()).andThen(new WaitCommand(0.5)).andThen(() -> {
                m_ShooterSubsystem.stopRollers(true);
                m_ShooterSubsystem.intake();
            })))
        .andThen(new WaitUntilCommand(() -> m_ShooterSubsystem.holdingNote == true))
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(() -> m_ShooterSubsystem.stopRollers(false)));*/
        
        
    /*return new InstantCommand(() -> {m_ShooterSubsystem.tempSetSpeed(0.45);
            m_ShooterSubsystem.setMidRollers(ShooterConstants.kMidRollerGrabSpeed);
        }, m_ShooterSubsystem)
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(() -> m_ShooterSubsystem.kickNote(false), m_ShooterSubsystem))
        .andThen(new WaitUntilCommand(() -> !m_ShooterSubsystem.hasNote()))
        .andThen(new WaitCommand(0.5))
        .andThen(() -> {
            m_ShooterSubsystem.stopShooterRollers();
            m_ShooterSubsystem.intake();
        });*/
        //.andThen(swerveControllerCommand.raceWith(new WaitUntilCommand(m_ShooterSubsystem::intakeFallingEdge)));
    //return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive);
    /*return new RunCommand(() -> {
        LimelightResults results = LimelightHelpers.getLatestResults("limelight");
        if(results.targetingResults.targets_Fiducials.length > 0) {
            double angle = Math.atan(results.targetingResults.targets_Fiducials[0].getTargetPose_CameraSpace().getX()/results.targetingResults.targets_Fiducials[0].getTargetPose_CameraSpace().getZ());
            SmartDashboard.putNumber("angle", angle);
            m_robotDrive.drive(0, 0, angle/-2.0, false, false);
        }
        else {
            m_robotDrive.drive(0, 0, 0, false, false);
        }
    }, m_robotDrive);*/
    /*if(type.getSelected().equals(AutoType.Normal)) {
        if(piece.getSelected().equals(AutoPiece.Cube)) {
        }
        else {
            if(rotate.getSelected().equals(AutoRotate.CC)) {
            }
            else if(rotate.getSelected().equals(AutoRotate.None)) {
            }
            else {
                return new RunCommand(null);
            }
        }
    }
    else {*/
        //return new InstantCommand();
    //}
    /*else {
        if(piece.getSelected().equals(AutoPiece.Cube)) {
        }
        else {
            return new InstantCommand(() -> m_hiArm.moveToPosition(185), m_hiArm) //Balencing Cone
        }
    }*/
    

    // Run path following command, then stop at the end.
    //return new RunCommand(() -> m_robotDrive.tankMetersPerSecond(-0.5, -0.5), m_robotDrive).until(() -> m_robotDrive.getPose().minus(new Pose2d(-1, 0, new Rotation2d(0))).getX() <= 0).andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    //4.1146 meters Normal distance Now in OIConstants
    
  }

  /*public Command debugAutonCommand() {
    return new RunCommand(() -> m_robotDrive.tankMetersPerSecond(1, 1), m_robotDrive);
  }*/

    /**
     * @param value
     * @param deadband
     * @param maxMagnitude
     * @return
     */
    private static double modifyAxis(double value, double deadband, double maxMagnitude) {
        
        // validate input
        double joyout = MathUtil.clamp(Math.abs(value),0.0,1.0);
        // apply deadband, scale remaining range 0..maxMagnitude
        joyout = MathUtil.applyDeadband(joyout, deadband, maxMagnitude);
        // square input, restore sign
        joyout = Math.copySign(joyout*joyout, value);

        return joyout;
    }
    private boolean leftTrigger() {
        return m_armController.getRawAxis(2) > 0.75;
    }
    private boolean rightTrigger() {
        return m_armController.getRawAxis(3) > 0.75;
    }
    private boolean R1Down() {
        return m_armController.getRawAxis(5) > 0.75;
    }
    private boolean R1Up() {
        return m_armController.getRawAxis(5) < -0.75;
    }
    private boolean L1Down() {
        return m_armController.getRawAxis(1) > 0.75;
    }
    private boolean L1Up() {
        return m_armController.getRawAxis(1) < -0.75;
    }

    public enum AutoType {
        One_Piece,
        Two_Piece,
        Three_Piece_Left,
        Three_Piece_Right,
        Four_Piece,
        Angled_Two_Piece
    }

    public enum AutoPiece {
        Cube,
        Cone
    }

    public enum AutoAngle {
        Right,
        Left
    }

    public enum AutoRotate {
        C,
        CC,
        None
    }

    //public void setBrakeMode(IdleMode mode) {
    //    m_robotDrive.setBrakeMode(mode);
    //}
    public void setFieldRelativeOffset(double offset) {
        m_robotDrive.setFieldRelativeOffset(offset);
    }
    public double testHeading() {
        return 5;
    }

    public ProfiledPIDController getThetaController() {
        ProfiledPIDController thetaController = new ProfiledPIDController(1.25, 0, 0, new TrapezoidProfile.Constraints(2*Math.PI, 2*Math.PI));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        return thetaController;
    }

    private Command backUpCommand(boolean reset) {
        Trajectory moveBackTraj = TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.3269, 5.553, new Rotation2d(0)),
            List.of(),
            centerNotePoint,
            AutoConstants.kTrajectoryConfig);
        SwerveControllerCommand moveBackPathCommand = new SwerveControllerCommand(moveBackTraj, 
            m_robotDrive::getPose, 
            Constants.DriveConstants.kDriveKinematics, 
            new PIDController(1, 0, 0), 
            new PIDController(1, 0, 0), 
            getThetaController(),
            /*() -> {
                double angle = -Math.atan((2.0 - m_robotDrive.getPose().getY())/m_robotDrive.getPose().getX());
                if(angle < 0) {
                    angle = Math.PI + angle;
                   }
               //return Rotation2d.fromRadians(angle);
                return Rotation2d.fromRadians(angle);
            },*/
            m_robotDrive::setModuleStates,
            m_robotDrive);
        if(reset) {
            m_robotDrive.resetOdometry(moveBackTraj.getInitialPose());
        }
        return moveBackPathCommand;
    }

    private Command moveToSpeakerCommand() {
        Trajectory moveToSpeakerTraj = TrajectoryGenerator.generateTrajectory(
            centerNotePoint,
            List.of(),
            new Pose2d(1.3269, 5.553, new Rotation2d(0)),
            AutoConstants.kTrajectoryConfigBackwards);
        SwerveControllerCommand moveToSpeakerCommand =  new SwerveControllerCommand(moveToSpeakerTraj, 
            m_robotDrive::getPose, 
            Constants.DriveConstants.kDriveKinematics, 
            new PIDController(1, 0, 0), 
            new PIDController(1, 0, 0), 
            getThetaController(),
            /*() -> {
                double angle = -Math.atan((2.0 - m_robotDrive.getPose().getY())/m_robotDrive.getPose().getX());
                if(angle < 0) {
                    angle = Math.PI + angle;
                }
                //return Rotation2d.fromRadians(angle);
                return Rotation2d.fromRadians(angle);
            },*/
            m_robotDrive::setModuleStates,
            m_robotDrive);
        return moveToSpeakerCommand;
    }

    private Command getOutCommand(boolean reset, AutoAngle side, Alliance alliance) {
        Trajectory traj;
        if(alliance.equals(Alliance.Blue)) {
            if(side.equals(AutoAngle.Left)) {
                traj = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0.686, 6.769, Rotation2d.fromDegrees(60)), 
                    List.of(new Translation2d(2.169, 7.719)), 
                    new Pose2d(3.918, 7.719, new Rotation2d()), 
                    AutoConstants.kTrajectoryConfig);
            }
            else {
                traj = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0.657, 4.350, Rotation2d.fromDegrees(-60)), 
                    List.of(new Translation2d(1.895, 2.274)), 
                    new Pose2d(3.856, 1.502, new Rotation2d()), 
                    AutoConstants.kTrajectoryConfig);
            }
        }
        else {
            if(side.equals(AutoAngle.Left)) {
                traj = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0.657, 3.85, Rotation2d.fromDegrees(60)), 
                    List.of(new Translation2d(1.895, 5.926)), 
                    new Pose2d(3.856, 6.698, new Rotation2d()), 
                    AutoConstants.kTrajectoryConfig);
            }
            else {
                traj = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0.657, 1.431, Rotation2d.fromDegrees(-60)), 
                    List.of(new Translation2d(1.895, 0.481)), 
                    new Pose2d(3.856, 0.481, new Rotation2d()), 
                    AutoConstants.kTrajectoryConfig);
            }
        }
        SwerveControllerCommand moveOut = new SwerveControllerCommand(traj, 
            m_robotDrive::getPose, 
            Constants.DriveConstants.kDriveKinematics, 
            new PIDController(1, 0, 0), 
            new PIDController(1, 0, 0), 
            getThetaController(),
            /*() -> {
                double angle = -Math.atan((2.0 - m_robotDrive.getPose().getY())/m_robotDrive.getPose().getX());
                if(angle < 0) {
                    angle = Math.PI + angle;
                   }
               //return Rotation2d.fromRadians(angle);
                return Rotation2d.fromRadians(angle);
            },*/
            m_robotDrive::setModuleStates,
            m_robotDrive);
        if(reset) {
            m_robotDrive.resetOdometry(traj.getInitialPose());
        }
        return moveOut;
    }

    private Command thirdNotePathLeft() {
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
            centerNotePoint,
            List.of(new Translation2d(1.3269, 5.553), new Translation2d(1.6269, 6.6)),
            new Pose2d(/*MAY NEED TO BE 2.8*/2.896, 7.2, new Rotation2d(-Math.PI + 0.463647609001)),
            AutoConstants.kTrajectoryConfigBackwards);
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(traj, 
        m_robotDrive::getPose, 
        Constants.DriveConstants.kDriveKinematics, 
        new PIDController(1, 0, 0), 
        new PIDController(1, 0, 0), 
        getThetaController(),
        this::autoPathAngleLeft,
        m_robotDrive::setModuleStates,
        m_robotDrive);
        return swerveControllerCommand;
    }

    private Command thirdNotePathRight() {
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
            centerNotePoint,
            List.of(new Translation2d(1.3269, 5.553), new Translation2d(1.6269, 4.506)),
            new Pose2d(/*MAY NEED TO BE 2.8*/2.896, 3.906, new Rotation2d(Math.PI - 0.463647609001)),
            AutoConstants.kTrajectoryConfigBackwards);
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(traj, 
        m_robotDrive::getPose, 
        Constants.DriveConstants.kDriveKinematics, 
        new PIDController(1, 0, 0), 
        new PIDController(1, 0, 0), 
        getThetaController(),
        this::autoPathAngleRight,
        m_robotDrive::setModuleStates,
        m_robotDrive);
        return swerveControllerCommand;
    }

    private Command fourthNothPathLeft() {
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
            new Pose2d(/*2.896*/2.896, 7.2, new Rotation2d(0.3)),
            List.of(),
            new Pose2d(/*2.896*/1.2, 5.553, new Rotation2d(Math.PI/2)),
            AutoConstants.kTrajectoryConfigBackwards);
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(traj, 
        m_robotDrive::getPose, 
        Constants.DriveConstants.kDriveKinematics, 
        new PIDController(1, 0, 0), 
        new PIDController(1, 0, 0), 
        getThetaController(),
        () -> new Rotation2d(0),
        m_robotDrive::setModuleStates,
        m_robotDrive);
        return swerveControllerCommand;
    }

    private Command fourthNothPathRight() {
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
            new Pose2d(/*2.896*/2.896, 3.906, new Rotation2d(-0.3)),
            List.of(),
            new Pose2d(/*2.896*/1.2, 5.553, new Rotation2d(-Math.PI/2)),
            AutoConstants.kTrajectoryConfigBackwards);
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(traj, 
        m_robotDrive::getPose, 
        Constants.DriveConstants.kDriveKinematics, 
        new PIDController(1, 0, 0), 
        new PIDController(1, 0, 0), 
        getThetaController(),
        () -> new Rotation2d(0),
        m_robotDrive::setModuleStates,
        m_robotDrive);
        return swerveControllerCommand;
    }

    private Rotation2d autoPathAngleLeft() {
        if(!atPlace) {
            //double angle = -Math.atan((5.553 - m_robotDrive.getPose().getY())/(m_robotDrive.getPose().getX()));
            /*if(angle < 0) {
                angle = Math.PI + angle;
            }*/
            return Rotation2d.fromRadians(0);
        }
        else {
            return new Rotation2d(0.3);
        }
    }

    private Rotation2d autoPathAngleRight() {
        if(!atPlace) {
            //double angle = -Math.atan((5.553 - m_robotDrive.getPose().getY())/(m_robotDrive.getPose().getX()));
            /*if(angle < 0) {
                angle = Math.PI + angle;
            }*/
            return Rotation2d.fromRadians(0);
        }
        else {
            return new Rotation2d(-0.3);
        }
    }

    private boolean atPlace() {
        Transform2d diff = m_robotDrive.getPose().minus(new Pose2d(1.3269, 5.553, new Rotation2d(0)));
        SmartDashboard.putString("diff", diff.toString());
        return (Math.abs(diff.getX()) < 0.3 && Math.abs(diff.getY()) < 1);
    }
}

