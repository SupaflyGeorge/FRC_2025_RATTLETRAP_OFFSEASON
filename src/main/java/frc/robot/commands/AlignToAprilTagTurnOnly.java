package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Rotate-in-place to center an AprilTag using Limelight's tx (degrees).
 * - Uses your SwerveSubsystem.drive(Translation2d, omegaRadPerSec, fieldRelative=true)
 * - Turns LEDs on while active; returns to pipeline control on end
 * - If no target, slowly "search" by spinning at a small rate
 */
public class AlignToAprilTagTurnOnly extends Command {
  private final SwerveSubsystem drive;
  private final String limelightName;
  private final NetworkTable ll;

  // PID on tx (deg). Start with small P and a touch of D; tune on-robot.
  private final PIDController pid = new PIDController(0.035, 0.0, 0.002);
  // how close is "aligned" (deg of tx)
  private final double toleranceDeg = 1.0;
  // clamp rotation command (rad/s) to something reasonable for your drive
  private final double maxOmegaRadPerSec = 2.5;
  // small search rate when no target (rad/s)
  private final double searchOmegaRadPerSec = 0.6;

  public AlignToAprilTagTurnOnly(SwerveSubsystem drive, String limelightName) {
    this.drive = drive;
    this.limelightName = limelightName;
    this.ll = NetworkTableInstance.getDefault().getTable(limelightName);
    pid.setSetpoint(0.0);
    pid.setTolerance(toleranceDeg);
    // tx is already in degrees; tell the PID that the input is degrees (so D is scaled right)
    pid.enableContinuousInput(-27.0, 27.0); // Limelight FOV (approx). Keeps behavior smooth near edges.
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Force LEDs on & assume pipeline 0 is an AprilTag pipeline (change if needed)
    ll.getEntry("pipeline").setNumber(0);
    ll.getEntry("ledMode").setNumber(3); // 3 = force on
  }

  @Override
  public void execute() {
    boolean hasTarget = ll.getEntry("tv").getDouble(0.0) > 0.5;
    double omegaCmd;

    if (hasTarget) {
      double txDeg = ll.getEntry("tx").getDouble(0.0);
      omegaCmd = pid.calculate(txDeg, 0.0);  // output in "deg units"; scale is fine since we clamp
      // Simple clamp to safe angular velocity
      omegaCmd = Math.max(-maxOmegaRadPerSec, Math.min(maxOmegaRadPerSec, omegaCmd));
      // If within tolerance, hold still
      if (pid.atSetpoint()) omegaCmd = 0.0;
    } else {
      // no target—slow search spin
      omegaCmd = searchOmegaRadPerSec;
    }

    // Rotate in place, field-relative (your drive() expects rad/s)
    drive.drive(new Translation2d(0.0, 0.0), omegaCmd, true);
  }

  @Override
  public void end(boolean interrupted) {
    // stop rotation & return LED control to pipeline
    drive.drive(new Translation2d(0.0, 0.0), 0.0, true);
    ll.getEntry("ledMode").setNumber(0); // 0 = pipeline
  }

  @Override
  public boolean isFinished() {
    // Keep running while held; if you want auto-finish when aligned, return pid.atSetpoint()
    return false;
  }
}
