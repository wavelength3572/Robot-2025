// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.Elastic;
import frc.robot.util.ReefScoringLogger;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private static final double loopOverrunWarningTimeout = 0.2;

  // Creating an empty HashMap
  Map<Integer, String> SparkCANIDMap = new HashMap<Integer, String>();

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Initialize URCL - Unofficial REV-Compatible Logger from AdvantageKit
    SparkCANIDMap.put(3, "DriveFrontRight");
    SparkCANIDMap.put(5, "DriveFrontLeft");
    SparkCANIDMap.put(6, "DriveBackRight");
    SparkCANIDMap.put(8, "DriveBackLeft");
    SparkCANIDMap.put(7, "TurnFrontRight");
    SparkCANIDMap.put(9, "TurnFrontLeft");
    SparkCANIDMap.put(2, "TurnBackRight");
    SparkCANIDMap.put(4, "TurnBackLeft");
    Logger.registerURCL(URCL.startExternal(SparkCANIDMap));

    // Start AdvantageKit logger
    Logger.start();

    // Adjust loop overrun warning timeout
    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(loopOverrunWarningTimeout);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    System.out.println(
        String.format("maxSpeedMetersPerSec: %.5f", DriveConstants.maxSpeedMetersPerSec));
    System.out.println(
        String.format("driveMotorReduction: %.5f", DriveConstants.driveMotorReduction));
    System.out.println(
        String.format("turnMotorReduction: %.5f", DriveConstants.turnMotorReduction));
    System.out.println(String.format("wheelRadiusMeters: %.5f", DriveConstants.wheelRadiusMeters));

    UsbCamera lifeCam = CameraServer.startAutomaticCapture("Climber Cam", 0);
    if (Constants.currentMode == Constants.Mode.REAL) {
      lifeCam.setResolution(416, 240);
    }
    FollowPathCommand.warmupCommand().schedule();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);

    if (robotContainer.getVisualizer() != null) {
      robotContainer.getVisualizer().update3DVisualization();
      robotContainer.getVisualizer().update2DVisualization();
    }

    if (robotContainer.getOdometryHealthMonitor() != null) {
      robotContainer.getOdometryHealthMonitor().checkOdometryHealth();
    }

    // Change to Climb Elastic Tab with 20 seconds left.
    if (DriverStation.getMatchType() != MatchType.None
        && DriverStation.isTeleop()
        && DriverStation.isEnabled()) {
      if (DriverStation.getMatchTime() <= 20) {
        Elastic.selectTab("Climb");
      }
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // robotContainer.getDrive().setPose(new Pose2d(7.157, 7.547, new
    // Rotation2d(Math.PI)));
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    robotContainer.updateOI(); // This only matters when the joysticks change
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    ReefScoringLogger.clearScoringEvents(); // Clears previous scoring data

    robotContainer
        .getCoralSystem()
        .autoSetHaveCoral(true); // Start with coral in robot during autonomous

    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    System.out.println("Boost Current Limit");

    robotContainer.getDrive().boostCurrentLimit();

    robotContainer.getVision().setVisionOn();

    // Set drive mode based on the boolean value
    if (robotContainer.getDrive().isDriveModeSmart() && DriverStation.isFMSAttached()) {
      robotContainer.SmartDriving();
    } else robotContainer.NormalDriving();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
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
  public void simulationInit() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
