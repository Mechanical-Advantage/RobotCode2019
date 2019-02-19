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
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;

public class Botlog{// Wrapper Class to do all things Badlog. If this creates errors you can blame me, Nicholas!
    
    private static BadLog log;
    private static long lastLog;

    //lists of outside Botlog trackings.
    private static List<botlogValue> values = new ArrayList<botlogValue>(0);
    
    private static List<botlogTopic> topics = new ArrayList<botlogTopic>(0);

    private static List<botlogTopicSub> topSubs = new ArrayList<botlogTopicSub>(0);

    private static int defaultUpdate = 250;

    //when Botlog is turned off, it will still make a .bag file to prevent errors,
    //but it won't be able to create/publish anything.
    public static void createBadlog(boolean runBotlog){//creates the Badlog
      File usb = new File("/media/sda1");//file location of the usb logs are stored on
      String date = LogUtil.genSessionName(); //gets the current date for naming the .bag file
      lastLog = System.currentTimeMillis();
      if(usb.exists() && usb.isDirectory() && usb.canRead() && usb.canWrite()){//conditional to check wheter ot not there is a usb to save to. default usb dir: /media/sda1.
        log = BadLog.init("/media/sda1/"  + date + ".bag");
        System.out.println("Sent to USB");

      } else{
        log = BadLog.init("/home/lvuser/Telemetry"  + date + ".bag");
        System.out.println("Sent to Roborio");
      }
        if(runBotlog){
          //Premade values, topics, and subscribers
          //Field
          BadLog.createValue("Match_Number", "" + DriverStation.getInstance().getMatchNumber());//example Value: key value-string pair known at init. Aka one time check.
          BadLog.createTopic("Match_Time", "s", () -> DriverStation.getInstance().getMatchTime()); //example Topic: constant stream of numeric data; what we're tracking
          //Joysticks/Buttons
          BadLog.createTopicSubscriber("Left_Joystick", BadLog.UNITLESS, DataInferMode.DEFAULT, ""); //example Subscriber: like a topic, but easier for tracking station input.
          BadLog.createTopicSubscriber("Right_Joystick", BadLog.UNITLESS, DataInferMode.DEFAULT, "");
          BadLog.createTopicSubscriber("Button_1", BadLog.UNITLESS, DataInferMode.DEFAULT, "");
          BadLog.createTopicSubscriber("Button_2", BadLog.UNITLESS, DataInferMode.DEFAULT, "");

          //outside values, topics, and subscribers made here. Requires that Robot.java is caled after 
          //EVERY class is run to add every terms.
          createValues();
          createTopics();
          createTopSubs();
        }
        log.finishInitialization();
    }

    public static void runPeriodic(){//caled in robot.periodic often, runs slower when disabled.
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
      if (!DriverStation.getInstance().isDisabled() || (currentMS - lastLog) >= defaultUpdate) {//makes code run slower when disabled; 1/4 the rate when enabled.
        lastLog = currentMS;
        BadLog.publish("Left_Joystick", Robot.oi.getLeftAxis());
        BadLog.publish("Right_Joystick", Robot.oi.getRightAxis());
        BadLog.publish("Button_1", sniper);
        BadLog.publish("Button_2", canDrive);
        //publishes the topics that were not manually created in Botlog
        publishSubs(currentMS - lastLog);
        log.updateTopics(); 
        log.log();
      }
      
    }

    //methods that allow outside classes to be tracked in BotLog. 
    //Make sure that you choose the right type, have ALL the required variables available,
    //and that what you are tracking is important.
    //check the premade examples above for a picture of what is needed. 
    public static void makeValue(String name, String source){//Creates a Value: one use check. For use outside of Botlog.
      botlogValue value = new botlogValue(name, source);
      values.add(value);
    }

    public static void makeTopic(String name, String units, Supplier source){//Creates a Topic: multiple use check from a supplier
      botlogTopic topic = new botlogTopic(name, units, source);
      topics.add(topic);
    }

    public static void makeTopicSub(String name, String unit, DataInferMode infer, String arg, Supplier source, boolean isDouble,  int time, boolean largo, boolean smol, double when){//Creates a Topic Subscriber: 
      botlogTopicSub sub = new botlogTopicSub(name, unit, infer, arg, source, isDouble, time, largo, smol, when); //multiple use that keeps a constant view on the source.
      topSubs.add(sub);
    }

    public static void createValues(){
      for (int x = 0; x < values.size(); x++) {
        BadLog.createValue(values.get(x).getName(), values.get(x).getSource());
      }
    }

    public static void createTopics(){
      for (int x = 0; x < topics.size(); x++) {
        BadLog.createTopic(topics.get(x).getName(), topics.get(x).getUnit(), topics.get(x).getSup());
      }
    }

    public static void createTopSubs(){
      for (int x = 0; x < topSubs.size(); x++) {
        BadLog.createTopicSubscriber(topSubs.get(x).getName(), topSubs.get(x).getUnit(), topSubs.get(x).getInfer(), topSubs.get(x).getArg());
      }
    }

    public static void publishSubs(double milis){//publishes ALL the topic subscribers.
      for (int x = 0; x < topSubs.size(); x++) {
        if(topSubs.get(x).pubTime >= milis && !DriverStation.getInstance().isDisabled()){//controls when the sub will publish based on time and wheter pr not the robot is enabled. If you want a topic sub to update outside dissables, define it inside Botlog itself.
          if(topSubs.get(x).getDouble()){
            if(topSubs.get(x).greater){
              if(topSubs.get(x).pubWhen > (double) topSubs.get(x).getSource().get()){//only publishes when the value is above the requirement
                BadLog.publish(topSubs.get(x).getName(), "" + topSubs.get(x).getSource().get());
              }
            } else if(topSubs.get(x).lesser){
              if(topSubs.get(x).pubWhen < (double) topSubs.get(x).getSource().get()){//only publishes when the value is below the requirement
                BadLog.publish(topSubs.get(x).getName(), "" + topSubs.get(x).getSource().get());
              }
            } else {
              BadLog.publish(topSubs.get(x).getName(), "" + topSubs.get(x).getSource().get());
            }
          } else {
            double isTrue;
            if(Robot.oi.getDriveEnabled()){
              isTrue = 1.0;
            } else {
              isTrue = 0.0;
            }
            if(topSubs.get(x).greater){
              if(topSubs.get(x).pubWhen > isTrue){
                BadLog.publish(topSubs.get(x).getName(), isTrue);
              }
            } else if(topSubs.get(x).lesser){
              if(topSubs.get(x).pubWhen < isTrue){
                BadLog.publish(topSubs.get(x).getName(), isTrue);
              }
            } else {
              BadLog.publish(topSubs.get(x).getName(), isTrue);
            }
          }
        }
      }
    }

    //below are objects that correlate to tgeir respective type.
    //made to keep the number of lists low.
    public static class botlogValue{//imagine the value
      private String vName;
      private String vSource;
      botlogValue(String name, String source){
        vName = name;
        vSource = source;
      }
      
      public String getName(){
        return vName;
      }

      public String getSource(){
        return vSource;
      }
    }

    public static class botlogTopic{//how topical
      private String tName;
      private String tUnit;
      private Supplier tSup;  
      public botlogTopic(String name, String unit, Supplier sup){
          tName = name;
          tUnit = unit;
          tSup = sup;
        }
        public String getName(){
          return tName;
        }

        public String getUnit(){
          return tUnit;
        }

        public Supplier getSup(){
          return tSup;
        }
    }

    public static class botlogTopicSub{//I have no joke for this
      private String TSName;
      private String TSUnit;
      private DataInferMode TSInfer;
      private String TSArg;
      private Supplier TSSource;
      private boolean TSIsDouble;
      private int pubTime = defaultUpdate;//time between pulishes, miliseconds
      private boolean greater;//will update only when output is greater than pubWhen
      private boolean lesser;//above but for less than. If neither, set both to false. NEVER SET BOTH TO TRUE.
      private double pubWhen;//sets conditional for publish
      botlogTopicSub(String name, String unit, DataInferMode infer, String arg, Supplier source, boolean isDouble, int time, boolean largo, boolean smol, double when){
        TSName = name;
        TSUnit = unit;
        TSInfer = infer;
        TSArg = arg;
        TSSource = source;
        TSIsDouble = isDouble;
        pubTime = time;
        greater = largo;
        lesser = smol;
        pubWhen = when;
      }

      public String getName(){
        return TSName;
      }

      public String getUnit(){
        return TSUnit;
      }

      public DataInferMode getInfer(){
        return TSInfer;
      }

      public String getArg(){
        return TSArg;
      }

      public Supplier getSource(){
        return TSSource;
      }

      public boolean getDouble(){
        return TSIsDouble;
      }
    }
}