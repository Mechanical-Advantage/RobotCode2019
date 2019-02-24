/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.StringJoiner;

import com.kauailabs.navx.frc.AHRS;

import org.zeromq.ZMQ;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveWithJoystick.JoystickMode;
import frc.robot.commands.ArmTuning;
import frc.robot.commands.DriveDistanceOnHeading;
import frc.robot.commands.FusedHeadingTest;
import frc.robot.commands.GenerateMotionProfiles;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.VelocityPIDTuner;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CameraSystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VisionData;
import frc.robot.subsystems.Vacuum;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final RobotMap robotMap = new RobotMap();

  public static ZMQ.Context ZMQContext = ZMQ.context(1);

  public static DriveTrain driveSubsystem /* = new DriveTrain()*/;
  public static final Arm arm = new Arm();
  public static final Vacuum vacuum = new Vacuum();
  public static final VisionData visionData = new VisionData();

  public static OI oi;
  public static final AHRS ahrs = new AHRS(SPI.Port.kMXP);

  public static final CameraSystem cameraSubsystem = new CameraSystem();

  Command autonomousCommand;
  SendableChooser<Command> tuningModeChooser = new SendableChooser<>();
  SendableChooser<AutoMode> autoChooser = new SendableChooser<>();
  public static SendableChooser<JoystickMode> joystickModeChooser;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    oi = new OI();
    joystickModeChooser = new SendableChooser<JoystickMode>();
    // chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    joystickModeChooser.setDefaultOption("Tank", JoystickMode.Tank);
    joystickModeChooser.addOption("Split Arcade", JoystickMode.SplitArcade);

    autoChooser.addOption("None", null);

    if (RobotMap.tuningMode) {
      tuningModeChooser.addOption("Fused Heading Test", new FusedHeadingTest());
      tuningModeChooser.addOption("Arm Tuning", new ArmTuning());
      // tuningModeChooser.addOption("Velocity PID Tuner", new VelocityPIDTuner());
      // tuningModeChooser.addOption("Turn 90 degrees", new TurnToAngle(90));
      // tuningModeChooser.addOption("Drive 5 feet", new DriveDistanceOnHeading(60));
      // tuningModeChooser.addOption("Drive 10 feet", new DriveDistanceOnHeading(120));
      SmartDashboard.putData("Tuning Auto Mode", tuningModeChooser);
      autoChooser.addOption("Tuning Auto", AutoMode.TUNING);
    }

    SmartDashboard.putData("Auto mode", autoChooser);
    SmartDashboard.putData("Control Mode", joystickModeChooser);

    // if the current waypoint version is old, re-generate profiles
    BufferedReader waypointVersionReader;
    int lastWaypointVersion = 0;
    try {
      waypointVersionReader = new BufferedReader(new FileReader("/home/lvuser/lastWaypointVersion"));
      lastWaypointVersion = Integer.parseInt(waypointVersionReader.readLine());
      waypointVersionReader.close();
    } catch (NumberFormatException | IOException e) {
      // do nothing
    }
    if (frc.robot.commands.GenerateMotionProfiles.waypointVersion > lastWaypointVersion) {
      GenerateMotionProfiles generateCommand = new GenerateMotionProfiles();
      generateCommand.setRunWhenDisabled(true);
      generateCommand.start();
    }
    Compressor c = new Compressor();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    /*if (driveSubsystem.getVelocityLeft() <= 2 && driveSubsystem.getVelocityRight() <= 2) {
      Robot.driveSubsystem.enableBrakeMode(false);
    }*/
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    // Robot.driveSubsystem.enableBrakeMode(true);
    ahrs.zeroYaw();
    if (autoChooser.getSelected() != null) {
      switch (autoChooser.getSelected()) {
        case TUNING:
          autonomousCommand = tuningModeChooser.getSelected();
      }
    }

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // Robot.driveSubsystem.enableBrakeMode(true);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    arm.setShoulderRaised(false);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  private enum AutoMode {
    TUNING;
  }

  // Utility functions
  public static String genGraphStr(double... data) {
    StringJoiner sj = new StringJoiner(":");
    for (double item : data) {
      sj.add(String.valueOf(item));
    }
    return sj.toString();
  }

  public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
}
