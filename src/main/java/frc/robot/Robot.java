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

import badlog.lib.BadLog;
import badlog.lib.DataInferMode;
import frc.robot.util.LogUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveWithJoystick.JoystickMode;
import frc.robot.commands.GenerateMotionProfiles;
import frc.robot.subsystems.CameraSystem;
import frc.robot.subsystems.DriveTrain;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final RobotMap robotMap = new RobotMap();

	public static final DriveTrain driveSubsystem = new DriveTrain();

  public static OI oi;
  public static final AHRS ahrs = new AHRS(SPI.Port.kMXP);

  public static final CameraSystem cameraSubsystem = new CameraSystem();

  Command autonomousCommand;
  SendableChooser<Command> tuningModeChooser = new SendableChooser<>();
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  public static SendableChooser<JoystickMode> joystickModeChooser;

  public static ZMQ.Context ZMQContext = ZMQ.context(1);

  BadLog log;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    oi = new OI();
    joystickModeChooser = new SendableChooser<JoystickMode>();
    // chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    joystickModeChooser.setDefaultOption("Tank", JoystickMode.Tank);
    joystickModeChooser.addOption("Split Arcade", JoystickMode.SplitArcade);

    if (RobotMap.tuningMode) {
      SmartDashboard.putData("Tuning Auto Mode", tuningModeChooser);
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
    //initiates BadLog, the tracking code that provides data from matches based on driver input.
    String date = LogUtil.genSessionName(); //gets the current date for naming the .bag file
    log = BadLog.init("/home/lvuser/"  + date + ".bag");

      BadLog.createValue("Match_Number", "" + DriverStation.getInstance().getMatchNumber()); //example Value: key value-string pair known at init.
      //Field
      BadLog.createTopic("Match_Time", "s", () -> DriverStation.getInstance().getMatchTime()); //example Topic: constant stream of numeric data; what we're tracking
      //Joysticks/Buttons
      BadLog.createTopicSubscriber("Left_Joystick", BadLog.UNITLESS, DataInferMode.DEFAULT, "xaxis"); //example Subscriber: takes info from Topics
      BadLog.createTopicSubscriber("Right_Joystick", BadLog.UNITLESS, DataInferMode.DEFAULT, "xaxis");
      BadLog.createTopicSubscriber("Button_1", BadLog.UNITLESS, DataInferMode.DEFAULT, "");
      BadLog.createTopicSubscriber("Button_2", BadLog.UNITLESS, DataInferMode.DEFAULT, "");
      //Robot Stuff
    log.finishInitialization();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    String lJoyInput = "" + Robot.oi.getLeftAxis();  //Badlog needs to recieve/send data constantly
    String rJoyInput = "" + Robot.oi.getRightAxis();
    String sniper = "" + Robot.oi.getSniperMode();
    String canDrive = "" + Robot.oi.getDriveEnabled();
    BadLog.publish("Left_Joystick", lJoyInput);
    BadLog.publish("Right_Joystick", rJoyInput);
    BadLog.publish("Button_1", sniper);
    BadLog.publish("Button_2", canDrive);
    log.updateTopics();
    log.log();

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    if (driveSubsystem.getVelocityLeft() <= 2 && driveSubsystem.getVelocityRight() <= 2) {
			Robot.driveSubsystem.enableBrakeMode(false);
		}
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    Robot.driveSubsystem.enableBrakeMode(true);
    autonomousCommand = autoChooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
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
    Robot.driveSubsystem.enableBrakeMode(true);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
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

  // Utility functions
	public static String genGraphStr(double...data) {
		StringJoiner sj = new StringJoiner(":");
		for (double item : data) {
			sj.add(String.valueOf(item));
		}
		return sj.toString();
	}
	
	public static double map(double x, double in_min, double in_max, double out_min, double out_max)
	{
	  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
}
