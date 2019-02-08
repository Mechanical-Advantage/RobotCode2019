/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import badlog.lib.BadLog;
import badlog.lib.DataInferMode;
import frc.robot.util.LogUtil;
import java.lang.Object;
import java.io.File;

import edu.wpi.first.wpilibj.DriverStation;

public class Botlog  {//wip class tp do all things Badlog. extends BadLog
   
    private BadLog log;
    private long lastLog;

    public void createBadlog(){
      File usb = new File("/media/sda1");
      String date = LogUtil.genSessionName(); //gets the current date for naming the .bag file
      lastLog = System.currentTimeMillis();
      if(usb.exists() && usb.isDirectory() && usb.canRead() && usb.canWrite()){//conditional to check wheter ot not there is a usb to save to. default usb dir: /media/sda1. useful info on https://docs.oracle.com/javase/7/docs/api/java/io/File.html
        log = BadLog.init("/media/sda1/"  + date + ".bag");
        System.out.println("Sent to USB");

      } else{
        log = BadLog.init("/home/lvuser/Telemetry"  + date + ".bag");
        System.out.println("Sent to Roborio");
      }
        //Field
        BadLog.createValue("Match_Number", "" + DriverStation.getInstance().getMatchNumber()); //example Value: key value-string pair known at init. Aka one time check.
        BadLog.createTopic("Match_Time", "s", () -> DriverStation.getInstance().getMatchTime()); //example Topic: constant stream of numeric data; what we're tracking
        //Joysticks/Buttons
        BadLog.createTopicSubscriber("Left_Joystick", BadLog.UNITLESS, DataInferMode.DEFAULT, ""); //example Subscriber: like a topic, but easier for tracking station input.
        BadLog.createTopicSubscriber("Right_Joystick", BadLog.UNITLESS, DataInferMode.DEFAULT, "");
        BadLog.createTopicSubscriber("Button_1", BadLog.UNITLESS, DataInferMode.DEFAULT, "");
        BadLog.createTopicSubscriber("Button_2", BadLog.UNITLESS, DataInferMode.DEFAULT, "");
        //Robot Stuff
      log.finishInitialization();
    }

    public void runPeriodic(){
        String lJoyInput = "" + Robot.oi.getLeftAxis();  //Badlog needs to recieve/send data constantly
    String rJoyInput = "" + Robot.oi.getRightAxis();
  
    double sniper;
    if(Robot.oi.getSniperMode()){
      sniper = 1.0;
    } else {
      sniper = 0.0;
    }

    double canDrive;
    if(Robot.oi.getDriveEnabled()){
      canDrive = 1.0;
    } else {
      canDrive = 0.0;
    }
    long currentMS = System.currentTimeMillis();
    if (!DriverStation.getInstance().isDisabled() || (currentMS - lastLog) >= 250) {
			lastLog = currentMS;
      BadLog.publish("Left_Joystick", lJoyInput);
      BadLog.publish("Right_Joystick", rJoyInput);
      BadLog.publish("Button_1", sniper);
      BadLog.publish("Button_2", canDrive);
      log.updateTopics(); 
      log.log();
    }
    }

}
