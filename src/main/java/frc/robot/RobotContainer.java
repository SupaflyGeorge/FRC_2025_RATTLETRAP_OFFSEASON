// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

import frc.robot.commands.ElevatorDown;
import frc.robot.commands.autocommands.AlgaeIntakeTimed;
import frc.robot.commands.autocommands.ElevatorDownWithIntake;
import frc.robot.commands.autocommands.ElevatorGoTo;
import frc.robot.commands.autocommands.PivotToPosition;
import frc.robot.commands.autocommands.PivotToPosition2;
import frc.robot.commands.ElevatorUp;
//import frc.robot.commands.IntakeCoral;
import frc.robot.commands.ElevatorSet;
import frc.robot.commands.AlignToAprilTagTurnOnly;
import frc.robot.commands.ClimbSet;

import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.PivotSet;
import frc.robot.commands.CoralPivotSet;
import frc.robot.commands.CoralIntakeRun;
import frc.robot.commands.CoralIntakeSafeDiag;
import frc.robot.commands.CoralIntakeVoltageTest;
import frc.robot.commands.CoralPivotSetPosition;

import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.mechanisms.AlgaeIntake;
import frc.robot.subsystems.mechanisms.Climb;
import frc.robot.subsystems.mechanisms.Pivot;
import frc.robot.subsystems.mechanisms.CoralIntake;
import frc.robot.subsystems.mechanisms.CoralPivot;
//import frc.robot.subsystems.mechanisms.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  public SendableChooser<Command> autoChooser;

  //Robots Subsystems and Commands are defined here
  public static final Elevator elevator = new Elevator();
  public static final Pivot pivot = new Pivot();
  public static final Climb climb = new Climb();
  public static final AlgaeIntake algaeIntake = new AlgaeIntake();
  public static final CoralIntake coralIntake = new CoralIntake();
  public static final CoralPivot coralPivot = new CoralPivot();
  

  // Top limit switch (DIO 9). If you ever rewire, adjust rawTopPressed().
  private final DigitalInput elevatorLimit = new DigitalInput(9); // DIO port

  // Latch: when true, UP is blocked until the top switch is RELEASED
  private boolean topLocked = false;

  // Debounce (~50ms) to avoid chatter at the top switch
  private final Debouncer topPressedDebounce  = new Debouncer(0.05, DebounceType.kRising);
  private final Debouncer topReleasedDebounce = new Debouncer(0.05, DebounceType.kFalling);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  // Example helpers available in your RobotContainer:
private boolean isUpBlocked() { 
  // Reuse your top lock / limit logic:
  return topLocked || isTopPressed(); // <- whatever you already use to block UP
}

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * 1,
                                                                () -> driverXbox.getLeftX() * 1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    


// Example: go down to L1 (= 0.00), run intake at +0.8, stop intake 1.0s after elevator reaches goal





    NamedCommands.registerCommand("PivotToDown",new PivotToPosition(pivot, 1.0, 7.5));
    NamedCommands.registerCommand("PivotToBall",new PivotToPosition(pivot, 1.0, 20));
    NamedCommands.registerCommand("PivotToUp",new PivotToPosition2(pivot, -1.0, 7.5));
    NamedCommands.registerCommand("PivotToScore",new PivotToPosition2(pivot, -1.0, 0.0));
NamedCommands.registerCommand(
  "AlgaeIntakeCool",
  new AlgaeIntakeTimed(algaeIntake, -1.0, 0.55) // run intake at full speed for 5 seconds
);


NamedCommands.registerCommand(
  "AlgaeIntakeNotCool",
  new AlgaeIntakeTimed(algaeIntake, 1.0, 1) // run intake at full speed for 5 seconds
);

    // after subsystems exist, in RobotContainer() constructor:

    // ...
    NamedCommands.registerCommand(
      "ElevatorDownWithIntake",
      new ElevatorDownWithIntake(
        elevator,
        algaeIntake,
        () -> 0.1,   // target position (example)
        0.8,          // down speed magnitude (applied as negative internally)
        0.01,         // tolerance (tune for your sensor units)
        1.0,          // intake speed
        1.0           // keep intake on for 1.0s after reaching goal
      )
    );
    NamedCommands.registerCommand(
      "ElevatorDownToBall",
      new ElevatorDownWithIntake(
        elevator,
        algaeIntake,
        () -> 0.25,   // target position (example)
        0.8,          // down speed magnitude (applied as negative internally)
        0.01,         // tolerance (tune for your sensor units)
        1.0,          // intake speed
        1.0           // keep intake on for 1.0s after reaching goal
      )
    );
    NamedCommands.registerCommand(
      "ElevatorToScore",
      new ElevatorGoTo(
        elevator,
        () -> 0.55,   // =
        0.8,          // up magnitude (internally +)
        0.8,          // down magnitude (internally -)
        0.01,
        this::isUpBlocked   
      )
    );
    
    NamedCommands.registerCommand(
      "ElevatorToL2",
      new ElevatorGoTo(
        elevator,
        () -> 0.95,   // L2 target
        0.8,
        0.8,
        0.5,
        this::isUpBlocked
      )
    );
    




    configureShuffleboard();
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = AutoBuilder.buildAutoChooser("Auto");
    //Calls the "Field" tab and adds list of Autonomous Routines----------------------------------------------------
    Shuffleboard.getTab("Field").add("Autonomous Routes", autoChooser);
  }

  private void configureShuffleboard() {
   // CameraServer.startAutomaticCapture(0);
   // CameraServer.putVideo("USB Camera 0", 200, 100);
  }

  // RAW read of the top limit.
  // Most FRC wiring makes DigitalInput.get() == false when PRESSED (pulled low).
  // So we invert here so rawTopPressed() is true when the switch is physically pressed.
  private boolean rawTopPressed() {
    return !elevatorLimit.get();
  }

  // Debounced edges for stable press/release triggers
  private boolean topPressedDebounced()  { return topPressedDebounce.calculate(rawTopPressed()); }
  private boolean topReleasedDebounced() { return topReleasedDebounce.calculate(rawTopPressed()); }

  // Convenience for gating UP (use raw—fast and fine here)
  private boolean isTopPressed() { return rawTopPressed(); }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@leink edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   * \
   */

   private static final double PIVOT_SAFE_DEG = 7.5;

  private void configureBindings()
  {

    //B -> Intakes the Coral-------------------------------------------------------
    /*operatorXbox.b().onTrue(new IntakeCoral(intake, .5));
    operatorXbox.b().onFalse(new IntakeCoral(intake, 0));
    //A -> OutTakes the Coral------------------------------------------------------
    operatorXbox.a().onTrue(new IntakeCoral(intake, -.5));
    operatorXbox.a().onFalse(new IntakeCoral(intake, 0));*/



    /*driverXbox.rightTrigger().whileTrue(
    new AlignToAprilTagTurnOnly(drivebase, "limelight")
    );*/

    //Right Trigger -> Extake Algae
    operatorXbox.rightTrigger().onTrue(new IntakeAlgae(algaeIntake, -1.0));
    operatorXbox.rightTrigger().onFalse(new IntakeAlgae(algaeIntake, 0));

    //Left Trigger -> Intake Algae
    operatorXbox.leftTrigger().onTrue(new IntakeAlgae(algaeIntake, 1.0));
    operatorXbox.leftTrigger().onFalse(new IntakeAlgae(algaeIntake,0));

    //X -> Pivot up
    operatorXbox.povUp().onTrue(new PivotSet(pivot, 0.90));
    operatorXbox.povUp().onFalse(new PivotSet(pivot, 0));

    //Down -> Pivot down
    operatorXbox.povDown().onTrue(new PivotSet(pivot, -0.90));
    operatorXbox.povDown().onFalse(new PivotSet(pivot, 0));

    driverXbox.x().whileTrue(new CoralIntakeRun(coralIntake, -0.35));
    driverXbox.b().whileTrue(new CoralIntakeRun(coralIntake, +0.15));  
    driverXbox.rightTrigger().whileTrue(new CoralPivotSet(coralPivot, +0.25));   
    driverXbox.leftTrigger().whileTrue(new CoralPivotSet(coralPivot, -0.3)); 
    
        //X -> Pivot up
       /*  driverXbox.povLeft().onTrue(new PivotSet(pivot, 0.75));
        driverXbox.povLeft().onFalse(new PivotSet(pivot, 0));
    
        //Down -> Pivot down
        driverXbox.povRight().onTrue(new PivotSet(pivot, -0.75));
        driverXbox.povRight().onFalse(new PivotSet(pivot, 0));*/

    //Pivot Setpoint (example)
    Command pivotZero = new PivotSet(pivot, (-1))
     .until (() -> pivot.getPosition() < 12.5);
    operatorXbox.povLeft().onTrue(pivotZero);

     //Pivot Setpoint (example)
     /*Command pivotZeroBack = new PivotSet(pivot, (1))
     .until (() -> pivot.getPosition() > 7.5);
    operatorXbox.povRight().onTrue(pivotZeroBack);*/

    //Right POV -> Climb up
    driverXbox.povUp().onTrue(new ClimbSet(climb, 1));
    driverXbox.povUp().onFalse(new ClimbSet(climb, 0));
 

    //Left POV -> Climb down
    driverXbox.povDown().onTrue(new ClimbSet(climb, -1));
    driverXbox.povDown().onFalse(new ClimbSet(climb, 0));

    // Manual: hold to move; releases -> PID holds there
    driverXbox.rightBumper().whileTrue(new ElevatorUp(elevator,  1.0));
    driverXbox.leftBumper().whileTrue(new ElevatorDown(elevator, 1.0));

// Setpoints: explicitly set goal AND enable PID
    driverXbox.a().onTrue(Commands.runOnce(() -> {
      elevator.setGoal(elevator.getTopMeters());
      elevator.enableClosedLoop();
  }));
  driverXbox.leftStick().whileTrue(new IntakeAlgae(algaeIntake, 1.0));
  driverXbox.rightStick().whileTrue(new IntakeAlgae(algaeIntake, -1.0));

 // public boolean atGoal() { return closedLoopEnabled && pid.atGoal(); }

// --- in configureBindings() ---
// In your RobotContainer.configureBindings():


// Create safety sequence for Y button (elevator down only if pivot is safe)
Command pivotToSafe =
  edu.wpi.first.wpilibj2.command.Commands
    .waitUntil(() -> pivot.getPosition() > PIVOT_SAFE_DEG)
    .deadlineWith(new PivotSet(pivot, +1.0))   // run pivot upward
    .withTimeout(2.5)
    .andThen(edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> pivot.set(0)));

Command elevatorDownPID =
  edu.wpi.first.wpilibj2.command.Commands
    .startEnd(
      () -> {
        elevator.setGoal(elevator.getBottomMeters());
        elevator.enableClosedLoop();
      },
      () -> {
        elevator.disableClosedLoop();
        elevator.stop();
      },
      elevator
    )
    .until(() -> elevator.atGoal());

// Combine: if pivot not safe, raise it first; else go straight down
Command ySequence =
  edu.wpi.first.wpilibj2.command.Commands.sequence(
    edu.wpi.first.wpilibj2.command.Commands.either(
      pivotToSafe,
      edu.wpi.first.wpilibj2.command.Commands.none(),
      () -> pivot.getPosition() < PIVOT_SAFE_DEG
    ),
    elevatorDownPID
  );

// Bind it to your Y button
driverXbox.y().onTrue(ySequence);




    

    // =========================
    // ELEVATOR MANUAL CONTROL + TOP LIMIT LATCH (PRESS to latch, RELEASE to clear)
    // =========================

    // A) Latch & STOP when the top limit is PRESSED (debounced)
    new Trigger(this::topPressedDebounced).onTrue(
      Commands.runOnce(() -> {
        topLocked = true;
        elevator.stop();
        System.out.println("[Elevator] TOP LIMIT PRESSED -> latched & stopped");
      }, elevator)
    );

    // B) CLEAR the latch when the top limit is RELEASED (debounced)
    new Trigger(this::topReleasedDebounced).onTrue(
      Commands.runOnce(() -> {
        if (topLocked) {
          topLocked = false;
          System.out.println("[Elevator] Top latch CLEARED (switch released)");
        }
      })
    );

    
    // release = stop

    // =========================
    // DRIVETRAIN BINDS (unchanged)
    // =========================  

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      //driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      //driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
      // () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

      // driverXbox.b().whileTrue(
      //   drivebase.driveToPose(
      //       new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      // );
    }
    /*if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }
*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}

