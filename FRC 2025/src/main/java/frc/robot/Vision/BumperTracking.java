package frc.robot.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

public class BumperTracking extends SubsystemBase {
  private PhotonCamera photonCamera;
  private VideoCapture videoCapture;
  private Scalar lowerColorBound;
  private Scalar upperColorBound;
  private boolean isRedAlliance;

  public BumperTracking() {
    photonCamera = new PhotonCamera("Cam1");
    videoCapture = new VideoCapture(0); // Assuming the camera index is 0
    setAllianceColor(true); // Default to red alliance
  }

  @Override
  public void periodic() {
    // Process AprilTags with PhotonVision
    PhotonPipelineResult result = photonCamera.getLatestResult();
    if (result.hasTargets()) {
      // Handle AprilTag detection
    }

    // Process bumper tracking with OpenCV
    Mat frame = new Mat();
    if (videoCapture.read(frame)) {
      Mat hsvFrame = new Mat();
      Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

      Mat mask = new Mat();
      Core.inRange(hsvFrame, lowerColorBound, upperColorBound, mask);

      // Find contours
      java.util.List<MatOfPoint> contours = new java.util.ArrayList<>();
      Mat hierarchy = new Mat();
      Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

      for (MatOfPoint contour : contours) {
        Rect rect = Imgproc.boundingRect(contour);
        Mat roi = frame.submat(rect);
        Mat grayRoi = new Mat();
        Imgproc.cvtColor(roi, grayRoi, Imgproc.COLOR_BGR2GRAY);
        Mat whiteMask = new Mat();
        Core.inRange(grayRoi, new Scalar(200), new Scalar(255), whiteMask);

        double whiteArea = Core.countNonZero(whiteMask);
        double contourArea = Imgproc.contourArea(contour);

        if (whiteArea / contourArea > 0.5) {
          // Detected a valid bumper
          Imgproc.rectangle(frame, rect, new Scalar(0, 255, 0), 2);
        }
      }

      // Display the processed frame (optional)
      // HighGui.imshow("Bumper Tracking", frame);
      // HighGui.waitKey(1);
    }
  }

  public void setAllianceColor(boolean isRed) {
    isRedAlliance = isRed;
    if (isRedAlliance) {
      lowerColorBound = new Scalar(0, 100, 100); // Adjust these values for red
      upperColorBound = new Scalar(10, 255, 255);
    } else {
      lowerColorBound = new Scalar(100, 100, 100); // Adjust these values for blue
      upperColorBound = new Scalar(140, 255, 255);
    }
  }
}