// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.stream.BaseStream;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.CC4HTriangulationImplementation;
import edu.wpi.first.math.geometry.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");

  private final CANSparkMax _leftLeadMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax _leftFollowMotor = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax _rightLeadMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax _rightFollowMotor = new CANSparkMax(2, MotorType.kBrushless);
  private final DifferentialDrive _drive = new DifferentialDrive(_leftLeadMotor, _rightLeadMotor);

  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  final double ANGULAR_P = 0.09;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

            /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    _leftLeadMotor.setSmartCurrentLimit(40);
    _leftFollowMotor.setSmartCurrentLimit(40);
    _rightLeadMotor.setSmartCurrentLimit(40);
    _rightFollowMotor.setSmartCurrentLimit(40);
        
        
        //Set lead and follow motors
    _leftFollowMotor.follow(_leftLeadMotor);
    _rightFollowMotor.follow(_rightLeadMotor);

            //Left side needs inverted
    _leftLeadMotor.setInverted(true);
    _rightLeadMotor.setInverted(false);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    double rotationSpeed = 0.0;
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      Transform3d targetPose = bestTarget.getBestCameraToTarget();
      //List<TargetCorner> corners = bestTarget.getDetectedCorners();

      SmartDashboard.putString("Target Status", "Target detected.");
      SmartDashboard.putNumber("Target X", targetPose.getTranslation().getX());
      SmartDashboard.putNumber("Target Y", targetPose.getTranslation().getY());
      SmartDashboard.putNumber("Target Z", targetPose.getTranslation().getZ());

      SmartDashboard.putNumber("Target Pitch", bestTarget.getPitch());
      SmartDashboard.putNumber("Target Yaw", bestTarget.getYaw()); // We were converting degrees to degrees lmao
      SmartDashboard.putNumber("Target Skew (?)", bestTarget.getSkew()); // Hopefully, this is roll.  Update: It was not roll.  We have no idea what this is.

      SmartDashboard.putNumber("AprilTag ID", bestTarget.getFiducialId());

      rotationSpeed = turnController.calculate(bestTarget.getPitch() / 3.5, 0);

      //placeholder shenanigans
      Translation2d absolute1 = new Translation2d(3.0, 7.0); //placeholder values, should have exact AprilTag coordinates
      Translation2d absolute2 = new Translation2d(0.0, 9.0); //placeholder values, should have exact AprilTag coordinates

      double flat1 = 7.0; //more placeholder!!!
      double flat2 = 10.0; //even more placeholder!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

      double yaw1 = 1.0; //placeholder dont panic
      double yaw2 = 2.0; //placeholder do panic

      CC4HTriangulationImplementation.TriangulationInputInfo info1 = new CC4HTriangulationImplementation().NastyInputInfoCavemanBrainedHack(absolute1, flat1, yaw1);
      CC4HTriangulationImplementation.TriangulationInputInfo info2 = new CC4HTriangulationImplementation().NastyInputInfoCavemanBrainedHack(absolute2, flat2, yaw2);

      new CC4HTriangulationImplementation().calculateTriangulationVector(info1, info2);

    } else {
      SmartDashboard.putString("Target Status", "No target detected.");
    }

    _drive.arcadeDrive(0, rotationSpeed);

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  } 

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
