// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.time.Instant;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ShootWhileMovingCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
  //private final HiArmSubsystem m_hiArm = new HiArmSubsystem();
  //private final LoArmSubsystem m_loArm = new LoArmSubsystem(m_hiArm);

  private Trajectory test_traj;
  private ShootWhileMovingCommand shootWhileCommand;

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
    /*SendableChooser<AutoType> autoType = new SendableChooser<AutoType>();
    autoType.addOption("Normal", AutoType.Normal);
    autoType.addOption("Balence", AutoType.Balence);
    autoType.setDefaultOption("Normal", AutoType.Normal);
    SendableChooser<AutoPiece> autoPiece = new SendableChooser<AutoPiece>();
    autoPiece.addOption("Cone", AutoPiece.Cone);
    autoPiece.addOption("Cube", AutoPiece.Cube);
    autoPiece.setDefaultOption("Cube", AutoPiece.Cube);
    SendableChooser<AutoRotate> autorotate = new SendableChooser<AutoRotate>();
    autorotate.addOption("Clockwise", AutoRotate.C);
    autorotate.addOption("Counter Clockwise", AutoRotate.CC);
    autorotate.addOption("None", AutoRotate.None);
    autorotate.setDefaultOption("None", AutoRotate.None);
    SmartDashboard.putNumber("Normal Auto Distance", AutoConstants.normalAutoDistance);
    SmartDashboard.putNumber("Balencing Auto Distance", AutoConstants.balenceAutoDistance);
    SmartDashboard.putData("Auto Type", autoType);
    SmartDashboard.putData("Auto Piece", autoPiece);
    SmartDashboard.putData("Auto Rot", autorotate);
    SmartDashboard.putNumber("limelight constant", 25);
    SmartDashboard.putNumber("limelight kp", 0.15);*/
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("testcurve.wpilib.json");
        test_traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        test_traj = new Trajectory();
    }
    SmartDashboard.putString("start", test_traj.getInitialPose().toString());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    /*new JoystickButton(m_armController, Button.kRightBumper.value)
        .onTrue(
            new ConditionalCommand(
                new InstantCommand(() -> m_loArm.moveToPosition(0)), 
                new InstantCommand(() -> m_hiArm.moveToPosition(140), m_hiArm)
                    .andThen(new WaitUntilCommand(m_hiArm::atPoint))
                    .andThen(new InstantCommand(() -> m_loArm.moveToPosition(0), m_hiArm, m_loArm)), 
                m_hiArm::inSafeZone));
        //.onTrue();
        //.onTrue(new InstantCommand(() -> m_loArm.moveToPosition(0)));
    new Trigger(this::rightTrigger)
        .onTrue(
            new ConditionalCommand(
                new InstantCommand(() -> m_loArm.moveToPosition(105)), 
                new InstantCommand(() -> m_hiArm.moveToPosition(140), m_hiArm)
                    .andThen(new WaitUntilCommand(m_hiArm::atPoint))
                    .andThen(new InstantCommand(() -> m_loArm.moveToPosition(105), m_hiArm, m_loArm)), 
                m_hiArm::inSafeZone));
        //.onTrue(new InstantCommand(() -> m_loArm.moveToPosition(90)));
    
    m_armControllerCommand.povUp().onTrue(new InstantCommand(m_hiArm::incrementArm, m_hiArm));
    m_armControllerCommand.povDown().onTrue(new InstantCommand(m_hiArm::decrementArm, m_hiArm));

    new JoystickButton(m_armController, Button.kA.value)
        .onTrue(new InstantCommand(() -> LedSubsystem.toggle_cube()));

    new JoystickButton(m_armController, Button.kY.value)
        .onTrue(new InstantCommand(() -> LedSubsystem.toggle_cone()));

    //new JoystickButton(m_driverController, Button.kB.value)
    //    .onTrue(new InstantCommand(() -> m_robotDrive.setBrakeMode(IdleMode.kBrake)))
    //    .onFalse(new InstantCommand(() -> m_robotDrive.setBrakeMode(IdleMode.kCoast)));
    new JoystickButton(m_armController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> m_hiArm.moveToPosition(0)));*/
    //new Trigger(this::leftTrigger)
        //.onTrue(new InstantCommand(() -> m_hiArm.moveToPosition(185)));
    new JoystickButton(m_driverController, Button.kY.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
    // Led bar triggers
    new JoystickButton(m_driverController, Button.kA.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    //m_armControllerCommand.povUp().onTrue(new InstantCommand(() -> m_ShooterSubsystem.setShooterVelocity(100)));
    //m_armControllerCommand.povUp().onFalse(new InstantCommand(() -> m_ShooterSubsystem.stopShooterRollers()));
    m_armControllerCommand.povUp().onTrue(new InstantCommand(() -> m_ArmSubsystem.setState(ArmState.kUp), m_ArmSubsystem));
    m_armControllerCommand.povDown().onTrue(new InstantCommand(() -> m_ArmSubsystem.setState(ArmState.kDown), m_ArmSubsystem));
    m_armControllerCommand.leftBumper().onTrue(new InstantCommand(() -> m_ArmSubsystem.intakeNote()));
    m_armControllerCommand.rightBumper().onTrue(new InstantCommand(() -> m_ArmSubsystem.expellNote()));
    m_armControllerCommand.leftBumper().onFalse(new InstantCommand(() -> m_ArmSubsystem.stopRoller()));
    m_armControllerCommand.rightBumper().onFalse(new InstantCommand(() -> m_ArmSubsystem.stopRoller()));
    m_armControllerCommand.x().onTrue(new InstantCommand(() -> m_ShooterSubsystem.speedUp(ShooterConstants.kShooterSpeedNormal)));
    m_armControllerCommand.x().onFalse(new InstantCommand(() -> m_ShooterSubsystem.stopShooterRollers()));
    m_armControllerCommand.a().onTrue(new InstantCommand(() -> m_ShooterSubsystem.intake()));
    m_armControllerCommand.a().onFalse(new InstantCommand(() -> m_ShooterSubsystem.stopRollers(false)));

  }
    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_ShooterSubsystem.holdingNote = true;
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

    // // Create config for trajectory

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(1.3269, 5.553, new Rotation2d(0)),
        List.of(),
        new Pose2d(2.8, 5.553, new Rotation2d(0)),
        AutoConstants.kTrajectoryConfig);
    
    Trajectory exampleTrajectoryBack = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(1.3716, 0, new Rotation2d()), new Pose2d(0, 0, new Rotation2d())),
        AutoConstants.kTrajectoryConfigBackwards);


    m_robotDrive.zeroHeading();
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    //SendableChooser<AutoType> type = (SendableChooser) SmartDashboard.getData("Auto Type");
    //SendableChooser<AutoPiece> piece = (SendableChooser) SmartDashboard.getData("Auto Piece");
    //SendableChooser<AutoRotate> rotate = (SendableChooser) SmartDashboard.getData("Auto Rot");

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory, 
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
    SwerveControllerCommand swerveControllerCommandBack =  new SwerveControllerCommand(exampleTrajectoryBack, 
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
    Command moveBack = new InstantCommand();
    Command thirdNotePath = new InstantCommand();
    Command fourthNotePath = new InstantCommand();
    shootWhileCommand = new ShootWhileMovingCommand(m_ShooterSubsystem, m_robotDrive);
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
    return new InstantCommand(() -> m_ShooterSubsystem.speedUp(ShooterConstants.kShooterSpeedNormal))
        .andThen(new WaitUntilCommand(m_ShooterSubsystem::atSpeed))
        .andThen(() -> m_ShooterSubsystem.kickNote(false))
        .andThen(new WaitUntilCommand(() -> !m_ShooterSubsystem.holdingNote))
        .andThen(new WaitCommand(0.25))
        .andThen(() -> m_ShooterSubsystem.stopRollers(true))
        .andThen(() -> m_ShooterSubsystem.setShooterVelocity(0))
        .andThen(() -> m_ShooterSubsystem.intake())
        .andThen(swerveControllerCommand)
        .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive)
        .andThen(() -> {
            m_ShooterSubsystem.stopRollers(false);
            m_ShooterSubsystem.setMidRollers(ShooterConstants.kMidRollerGrabSpeed);
            })
        .andThen(new WaitCommand(0.5))
        .andThen(new ParallelRaceGroup(thirdNotePath(), shootWhileCommand))
        .andThen(() -> m_ShooterSubsystem.stopRollers(false))
        .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive)
        .andThen(() -> m_ShooterSubsystem.setMidRollers(ShooterConstants.kMidRollerGrabSpeed))
        .andThen(new ParallelCommandGroup(fourthNothPath(), new WaitCommand(0.5).andThen(() -> m_ShooterSubsystem.speedUp(ShooterConstants.kShooterSpeedNormal + 250))))
        .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive)
        .andThen(() -> m_ShooterSubsystem.kickNote(false));

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
    /*private boolean leftTrigger() {
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
    }*/

    public enum AutoType {
        Normal,
        Balence
    }

    public enum AutoPiece {
        Cube,
        Cone
    }

    public enum AutoRotate {
        C,
        CC,
        None
    }

    //public void setBrakeMode(IdleMode mode) {
    //    m_robotDrive.setBrakeMode(mode);
    //}
    public void setSubsystemAuto(boolean auto) {
        //m_hiArm.auto = auto;
    }
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

    private Command thirdNotePath() {
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
            new Pose2d(2.8, 5.553, new Rotation2d(0)),
            List.of(new Translation2d(1.3269, 5.553), new Translation2d(1.6269, 6.6)),
            new Pose2d(/*2.896*/2.8, 7.2, new Rotation2d(-Math.PI + 0.463647609001)),
            AutoConstants.kTrajectoryConfigBackwards);
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(traj, 
        m_robotDrive::getPose, 
        Constants.DriveConstants.kDriveKinematics, 
        new PIDController(1, 0, 0), 
        new PIDController(1, 0, 0), 
        getThetaController(),
        this::autoPathAngle,
        m_robotDrive::setModuleStates,
        m_robotDrive);
        return swerveControllerCommand;
    }

    private Command fourthNothPath() {
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
            new Pose2d(/*2.896*/2.8, 7.2, new Rotation2d(0.3)),
            List.of(),
            new Pose2d(/*2.896*/1.2, 5.553, new Rotation2d(0)),
            AutoConstants.kTrajectoryConfigBackwards);
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(traj, 
        m_robotDrive::getPose, 
        Constants.DriveConstants.kDriveKinematics, 
        new PIDController(1, 0, 0), 
        new PIDController(1, 0, 0), 
        getThetaController(),
        m_robotDrive::setModuleStates,
        m_robotDrive);
        return swerveControllerCommand;
    }

    private Rotation2d autoPathAngle() {
        if(!shootWhileCommand.atPlace) {
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
}

