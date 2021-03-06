package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.RobotMap.RobotType;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Controls front and back cameras
 */
public class CameraSystem extends Subsystem {

  private UsbCamera frontCamera;
  private UsbCamera secondCamera;
  private boolean serverCreated = false;
  private boolean frontCameraAdded = false;
  private boolean secondCameraAdded = false;
  private int frontCameraID;
  private int secondCameraID;

  private static final int compressionQuality = 50; // Valid range 0-100, higher value for higher quality and higher

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public CameraSystem() {
    switch (RobotMap.robot) {
    case ROBOT_2017:
      frontCameraID = 2;
      secondCameraID = 0;
      break;
    case ORIGINAL_ROBOT_2018:
      frontCameraID = 0;
      secondCameraID = 2;
      break;
    case ROBOT_2019:
    case ROBOT_2019_2:
      frontCameraID = 0;
      secondCameraID = 2;
      break;
    case EVERYBOT_2019:
    default:
      break;
    }
  }

  // To get network tables publishing, must use startAutomaticCapture and have it
  // create the UsbCamera.
  // This should not have to work this way, but CameraServer is finicky, and this
  // was the only way it worked
  // Written for WPILib 2017.3.1
  // Note: Second Camera is just internal, selecting on dashboard will have no
  // effect
  private UsbCamera setupServer(int id) {
    serverCreated = true;
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture("Video Feed", id);
    ((MjpegServer) CameraServer.getInstance().getServer()).setCompression(compressionQuality);
    return camera;
  }

  public void useFrontCamera() {
    if (!frontCameraAdded) {
      if (!serverCreated) {
        frontCamera = setupServer(frontCameraID);
      } else {
        frontCamera = new UsbCamera("Second Camera", frontCameraID);
      }
      frontCamera.setResolution(320, 240);
      frontCamera.setFPS(15);
      if (RobotMap.robot == RobotType.ROBOT_2019 || RobotMap.robot == RobotType.ROBOT_2019_2) {
        frontCamera.setBrightness(30); // For MS Lifecam 3000 because it defaults to a very low brightness
      }
      frontCameraAdded = true;
    }
    CameraServer.getInstance().getServer().setSource(frontCamera);
    System.out.println("Switching to front camera");
  }

  public void useSecondCamera() {
    if (RobotMap.robot == RobotType.ROBOT_2017 || RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
      if (!secondCameraAdded) {
        if (!serverCreated) {
          secondCamera = setupServer(secondCameraID);
        } else {
          secondCamera = new UsbCamera("Second Camera", secondCameraID);
        }
        secondCamera.setResolution(320, 240);
        secondCamera.setFPS(15);
        secondCameraAdded = true;
      }
      CameraServer.getInstance().getServer().setSource(secondCamera);
      System.out.println("Switching to second camera");
    }
  }
}
