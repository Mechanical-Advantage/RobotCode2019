/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.zeromq.ZMQ;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;

/**
 * Communicates with the coprocessor to recieve vision data
 */
public class VisionData extends Subsystem {

  private static final String commandAddress = "tcp://frcvision.local:5555";
  private static final String senderAddress = "tcp://frcvision.local:5556";

  public HatchPipeline hatch = new HatchPipeline();
  public DeliveryTargetPipeline delivery = new DeliveryTargetPipeline();
  private final Pipeline[] pipelines = new Pipeline[] {hatch, delivery};

  ZMQ.Socket commandSocket = Robot.ZMQContext.socket(ZMQ.PUSH);
  ZMQ.Socket recieverSocket = Robot.ZMQContext.socket(ZMQ.SUB);
  private Pipeline currentPipeline;

  public VisionData() {
    commandSocket.connect(commandAddress);
    recieverSocket.connect(senderAddress);
    // Subscripe to everything that a pipeline requires
    // Changing subscriptions dynamically risks dropping messages
    for (Pipeline pipeline : pipelines) {
      for (byte[] subscription : pipeline.getSubscriptions()) {
        recieverSocket.subscribe(subscription);
      }
    }
  }

  public void periodic() {
    byte[] firstFrame = recieverSocket.recv(ZMQ.NOBLOCK);
    if (firstFrame != null) {
      List<byte[]> frames = new ArrayList<byte[]>();
      frames.add(firstFrame);
      while (recieverSocket.hasReceiveMore()) {
        frames.add(recieverSocket.recv());
      }
      if (currentPipeline != null) {
        currentPipeline.processData(frames);
      }
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void stopPipeline() {
    setPipeline(null);
  }

  public void setPipeline(Pipeline pipeline) {
    commandSocket.send("set_pipeline", ZMQ.SNDMORE);
    if (pipeline != null) {
      commandSocket.send(pipeline.getName());
    } else {
      commandSocket.send("none");
    }
    currentPipeline = pipeline;
  }

  /**
   * Base class for all pipelines, implements timestamp recieving
   */
  public abstract class Pipeline {
    protected double lastTimestamp;
    // This means that the 0s from before the first data point are "handled" 
    // so the user of the pipeline does not try to process the invalid data
    protected boolean currentDataHandled = true;

    public abstract String getName();
    public abstract byte[][] getSubscriptions();
    protected void processData(List<byte[]> frames) {
      lastTimestamp = ByteBuffer.wrap(frames.get(1)).getDouble();
      currentDataHandled = false;
    }
    /**
     * @return the timestamp of the last data recieved
     */
    public double getLastTimestamp() {
      return lastTimestamp;
    }

    /**
     * Sets a flag that the current data point has been processed.
     * This flag resets when new data is recieved.
     */
    public void dataHandled() {
      currentDataHandled = true;
    }

    public boolean isDataHandled() {
      return currentDataHandled;
    }
  }

  public class HatchPipeline extends Pipeline {
    private double distance;
    private double angle;

    private HatchPipeline() {};

    @Override
    public String getName() {
      return "hatch";
    }

    @Override
    public byte[][] getSubscriptions() {
      /*
      * frame 0: "distangle"
      * frame 1: timestamp (double)
      * frame 2: distance to target (double)
      * frame 3: angle to target (double)
      */
      return new byte[][] {"distangle".getBytes()};
    }

    @Override
    protected void processData(List<byte[]> frames) {
      if (Arrays.equals(frames.get(0), "distangle".getBytes())) {
        super.processData(frames);
        distance = ByteBuffer.wrap(frames.get(2)).getDouble();
        angle = ByteBuffer.wrap(frames.get(3)).getDouble();
        lastTimestamp = Timer.getFPGATimestamp(); // Temporary until real timestamps
      }
    }

    /**
     * @return the last distance recieved
     */
    public double getDistance() {
      return distance;
    }

    /**
     * @return the last angle recieved
     */
    public double getAngle() {
      return angle;
    }
  }

  public class DeliveryTargetPipeline extends Pipeline {
    private double x;
    private double y;
    private double angle;

    private DeliveryTargetPipeline() {};

    @Override
    public String getName() {
      return "delivery";
    }

    @Override
    public byte[][] getSubscriptions() {
      /*
      * frame 0: "coordinates"
      * frame 1: timestamp (double)
      * frame 2: x from target (double)
      * frame 3: y from target (double)
      * frame 4: angle from robot to target (double)
      */
      return new byte[][] {"coordinates".getBytes()};
    }

    @Override
    protected void processData(List<byte[]> frames) {
      if (frames.get(0) == "coordinates".getBytes()) {
        super.processData(frames);
        x = ByteBuffer.wrap(frames.get(2)).getDouble();
        y = ByteBuffer.wrap(frames.get(3)).getDouble();
        angle = ByteBuffer.wrap(frames.get(4)).getDouble();
      }
    }

    /**
     * @return the last x recieved
     */
    public double getX() {
      return x;
    }

    /**
     * @return the last y recieved
     */
    public double getY() {
      return y;
    }

    /**
     * @return the last angle reciever
     */
    public double getAngle() {
      return angle;
    }
  }
}
