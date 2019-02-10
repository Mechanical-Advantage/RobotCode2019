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
import java.io.File;
import java.util.*;

import edu.wpi.first.wpilibj.DriverStation;

public class Botlog{// Wrapper Class to do all things  Badlog.
   
    private static BadLog log;
    private static long lastLog;
    private List<String> topicSubNames;
    private List<String> topicSubSource;

    public static void createBadlog(){//creates the Badlog
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

    public static void runPeriodic(){//caled in robot.periodic often, runs slower when disabled.
      String lJoyInput = "" + Robot.oi.getLeftAxis();
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
      if (!DriverStation.getInstance().isDisabled() || (currentMS - lastLog) >= 250) {//makes code run slower when disabled; 1/4 the rate when enabled.
        lastLog = currentMS;
        BadLog.publish("Left_Joystick", lJoyInput);
        BadLog.publish("Right_Joystick", rJoyInput);
        BadLog.publish("Button_1", sniper);
        BadLog.publish("Button_2", canDrive);
        log.updateTopics(); 
        log.log();
      }
    }

    public void makeValue(String name, String unit){

    }

    public void makeTopic(String name, String unit){
      
    }

    public void makeTopicSub(String name, Double recording, String unit){
      
    }

    public void makeTopicSub(String name, Boolean recording, String unit){
      
    }

}
