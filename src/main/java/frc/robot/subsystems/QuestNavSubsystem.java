package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import swervelib.SwerveDrive;

import java.util.ArrayList;
import java.util.List;

public class QuestNavSubsystem {
  private final QuestNav m_questNav = new QuestNav();
  private final SwerveDrive m_drivetrain;

  /**
   * Transform from robot center to the Quest headset mounting location.
   * Adjust x/y/z (meters) and yaw/pitch/roll (radians) to match your physical mounting.
   */
  private static final Transform3d ROBOT_TO_QUEST =
      new Transform3d(
          new Translation3d(Units.metersToInches(12.0), Units.metersToInches(0.0), Units.metersToInches(12)),
          new Rotation3d(0.0, 0.0, 0.0));

  private static final Matrix<N3, N1> QUESTNAV_STD_DEVS =
      VecBuilder.fill(
          0.02,       // X position trust (20mm)
          0.02,       // Y position trust (20mm)
          0.0872665); // Rotation trust (5 degrees)

  private static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  private static final double BATTERY_LOW_PERCENT = 20;
  private static final double BATTERY_CRITICAL_PERCENT = 10;

  // NT struct publishers for AdvantageScope 3D visualization
  private final StructArrayPublisher<Pose3d> m_allPosesPub;
  private final StructArrayPublisher<Pose3d> m_acceptedPosesPub;
  private final StructArrayPublisher<Pose3d> m_rejectedPosesPub;
  private final StructPublisher<Pose3d> m_latestPosePub;

  private double m_lastPoseTimestamp = -1;

  public QuestNavSubsystem(SwerveDrive drivetrain) {
    m_drivetrain = drivetrain;
    SmartDashboard.putBoolean("Odometry/Quest-Enable", false);

    var nt = NetworkTableInstance.getDefault();
    m_allPosesPub =
        nt.getStructArrayTopic("QuestNav/RobotPoses", Pose3d.struct).publish();
    m_acceptedPosesPub =
        nt.getStructArrayTopic("QuestNav/RobotPosesAccepted", Pose3d.struct).publish();
    m_rejectedPosesPub =
        nt.getStructArrayTopic("QuestNav/RobotPosesRejected", Pose3d.struct).publish();
    m_latestPosePub =
        nt.getStructTopic("QuestNav/LatestRobotPose", Pose3d.struct).publish();

    m_questNav.setVersionCheckEnabled(false);

    m_questNav.onConnected(() ->
        System.out.println("Quest connected!"));
    m_questNav.onDisconnected(() ->
        DriverStation.reportWarning("Quest disconnected!", false));
    m_questNav.onTrackingAcquired(() ->
        System.out.println("Quest tracking acquired!"));
    m_questNav.onTrackingLost(() ->
        DriverStation.reportWarning("Quest tracking lost!", false));
    m_questNav.onLowBattery((int) BATTERY_LOW_PERCENT, level ->
        DriverStation.reportWarning("Quest battery low: " + level + "%", false));
    m_questNav.onCommandSuccess(response ->
        System.out.println("Pose reset succeeded: " + response.getCommandId()));
    m_questNav.onCommandFailure(response ->
        DriverStation.reportError(
            "Pose reset failed: " + response.getErrorMessage(), false));
  }

  public void periodic() {
    m_questNav.commandPeriodic();

    // Publish device diagnostics
    boolean connected = m_questNav.isConnected();
    boolean tracking = m_questNav.isTracking();
    SmartDashboard.putBoolean("QuestNav/Connected", connected);
    SmartDashboard.putBoolean("QuestNav/Tracking", tracking);
    SmartDashboard.putNumber("QuestNav/Latency", m_questNav.getLatency());
    m_questNav.getBatteryPercent().ifPresent(b -> {
      SmartDashboard.putNumber("QuestNav/Battery%", b);
      if (b < BATTERY_CRITICAL_PERCENT) {
        DriverStation.reportWarning("Quest battery CRITICAL: " + b + "%", false);
      }
    });
    m_questNav.getTrackingLostCounter().ifPresent(
        c -> SmartDashboard.putNumber("QuestNav/TrackingLostCount", c));

    // Process all unread pose frames
    PoseFrame[] frames = m_questNav.getAllUnreadPoseFrames();
    SmartDashboard.putNumber("QuestNav/UnreadFrames", frames.length);

    List<Pose3d> allPoses = new ArrayList<>();
    List<Pose3d> acceptedPoses = new ArrayList<>();
    List<Pose3d> rejectedPoses = new ArrayList<>();

    for (PoseFrame frame : frames) {
      Pose3d questPose = frame.questPose3d();
      Pose3d robotPose = questPose.plus(ROBOT_TO_QUEST.inverse());

      allPoses.add(robotPose);
      if (shouldReject(robotPose)) {
        rejectedPoses.add(robotPose);
        continue;
      }

      acceptedPoses.add(robotPose);

      if (frame.isTracking() && SmartDashboard.getBoolean("QuestNav/UsePose", true)) {
        m_drivetrain.addVisionMeasurement(
            robotPose.toPose2d(), frame.dataTimestamp(), QUESTNAV_STD_DEVS);
      }
      Field2d odField = (Field2d)SmartDashboard.getData("Odometry/Field");
      FieldObject2d odFieldQuest = odField.getObject("Quest");
      odFieldQuest.setPose(robotPose.toPose2d());
      SmartDashboard.putData("Odometry/Field", odField);
        
      SmartDashboard.putNumber("Odometry/quest/X", robotPose.getX());
      SmartDashboard.putNumber("Odometry/quest/Y", robotPose.getY()); 
      SmartDashboard.putNumber("Odometry/quest/Heading", Units.radiansToDegrees(robotPose.getRotation().getAngle()));

      m_lastPoseTimestamp = frame.dataTimestamp();
    }

    // Publish pose arrays for AdvantageScope 3D field visualization
    m_allPosesPub.set(allPoses.toArray(Pose3d[]::new));
    m_acceptedPosesPub.set(acceptedPoses.toArray(Pose3d[]::new));
    m_rejectedPosesPub.set(rejectedPoses.toArray(Pose3d[]::new));

    if (!allPoses.isEmpty()) {
      m_latestPosePub.set(allPoses.get(allPoses.size() - 1));
    }

    if (m_lastPoseTimestamp > 0) {
      SmartDashboard.putNumber(
          "QuestNav/TimeSinceLastPose", Timer.getTimestamp() - m_lastPoseTimestamp);
    }
  }

  private boolean shouldReject(Pose3d pose) {
    return pose.getX() < 0.0
        || pose.getX() > FIELD_LAYOUT.getFieldLength()
        || pose.getY() < 0.0
        || pose.getY() > FIELD_LAYOUT.getFieldWidth();
  }

  public void resetPose(Pose3d robotPose) {
    Pose3d questPose = robotPose.plus(ROBOT_TO_QUEST);
    m_questNav.setPose(questPose);
    //m_questNav.setPose(new Pose3d(3,3,12, new Rotation3d(0,0,Math.toRadians(180))));
  }

  public void resetPose(Pose2d robotPose) {
    resetPose(new Pose3d(robotPose.getX(), robotPose.getY(),0, new Rotation3d(0, 0, robotPose.getRotation().getRadians())));
  }

}
