// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  // Motors
  private final SparkMax leftElevator  = new SparkMax(9,  MotorType.kBrushless);
  private final SparkMax rightElevator = new SparkMax(10, MotorType.kBrushless);

  // (kept for debug)
  private final RelativeEncoder armRightMotorEncoder = leftElevator.getEncoder();
  private final RelativeEncoder armLeftMotorEncoder  = rightElevator.getEncoder();

  // === EDIT THESE TO MATCH YOUR ROBOT ===
  private static final int    CANCODER_ID          = 15;      // your CANcoder ID
  private static final double DRUM_DIAMETER_METERS = 0.0381;  // e.g. 1.5 in OD
  private static final boolean UP_IS_POSITIVE      = true;    // flip if needed

  // Soft limits (meters)
  private final double bottomMeters = 0.0;
  private final double topMeters    = 0.627;   // put your real max

  // Absolute sensor
  private final CANcoder shaftEncoder = new CANcoder(CANCODER_ID);

  // ***** Closed-loop only for setpoints *****
  private static final TrapezoidProfile.Constraints kConstraints =
      new TrapezoidProfile.Constraints(1.6, 3.0);  // tune
  private final ProfiledPIDController pid =
      new ProfiledPIDController(4.0, 0.0, 0.0, kConstraints); // tune

  private boolean closedLoopEnabled = false;
  private double goalMeters = bottomMeters;

  // ===== Ramp control =====
  /** Seconds from 0→100% output; 0 disables (instant). */
  private double rampSeconds = 0.15;

  /** Software ramp (extra smoothing on top of REV firmware ramp). */
  private SlewRateLimiter slew = new SlewRateLimiter(Double.POSITIVE_INFINITY);

  public Elevator() {
    // Sensor setup
    shaftEncoder.getConfigurator().apply(new CANcoderConfiguration());

    // Reset debug encoders
    armRightMotorEncoder.setPosition(0);
    armLeftMotorEncoder.setPosition(0);

    // Default: no ramp; you can change at runtime via setRampSeconds()
    applyRampToSparks();

    // Shuffleboard debug
    Shuffleboard.getTab("Debug (Sensors)")
        .addDouble("Elevator Pos (m)", this::getPosition);
    Shuffleboard.getTab("Debug (Sensors)")
        .addDouble("Elevator Goal (m)", () -> goalMeters);
    Shuffleboard.getTab("Debug (Sensors)")
        .addBoolean("Elev PID Enabled", () -> closedLoopEnabled);
    Shuffleboard.getTab("Debug (Sensors)")
        .addDouble("Elev Ramp (s 0->1)", () -> rampSeconds);

    pid.setTolerance(0.005); // 5 mm
    pid.setGoal(goalMeters);
  }

  // ---- Conversions ----
  private static double metersPerRev() { return Math.PI * DRUM_DIAMETER_METERS; }

  // ---- Sensors ----
  public double getPosition() {
    double revs = shaftEncoder.getPosition().getValueAsDouble();
    double meters = revs * metersPerRev();
    return UP_IS_POSITIVE ? meters : -meters;
  }

  public double getVelocity() {
    double rps = shaftEncoder.getVelocity().getValueAsDouble();
    double mps = rps * metersPerRev();
    return UP_IS_POSITIVE ? mps : -mps;
  }

  // ---- Actuation ----
  /** Core motor setter with deadband, soft limits, and ramp. */
  private void setPercent(double speed) {
    // deadband
    if (Math.abs(speed) < 0.03) speed = 0.0;

    // soft limits
    double pos = getPosition();
    boolean wantsUp   = speed > 0;
    boolean wantsDown = speed < 0;
    if (pos >= topMeters && wantsUp)      speed = 0.0;
    if (pos <= bottomMeters && wantsDown) speed = 0.0;

    // clamp and ramp
    speed = MathUtil.clamp(speed, -1.0, 1.0);
    double smoothed = slew.calculate(speed);   // software ramp

    leftElevator.set(smoothed);
    rightElevator.set(smoothed);
  }

  /** Public manual set (open-loop). */
  public void set(double speed) { setPercent(speed); }

  public void stop() { leftElevator.set(0); rightElevator.set(0); }

  // ---- Setpoints / Modes ----
  public void setGoal(double meters) {
    goalMeters = Math.max(bottomMeters, Math.min(topMeters, meters));
    pid.setGoal(goalMeters);
  }

  public void enableClosedLoop()  { closedLoopEnabled = true;  }
  public void disableClosedLoop() { closedLoopEnabled = false; }

  public double getTopMeters()    { return topMeters; }
  public double getBottomMeters() { return bottomMeters; }

  // ---- Ramp controls (REV 2025 API + software slew) ----
  /**
   * Change how “soft” the throttle is.
   * @param seconds time to ramp from 0 → 100% output. 0 = instant.
   */
  public void setRampSeconds(double seconds) {
    rampSeconds = Math.max(0.0, seconds);
    applyRampToSparks();
    // Match software slew to the same rise/fall rate:
    double ratePerSec = (rampSeconds <= 1e-6) ? Double.POSITIVE_INFINITY : 1.0 / rampSeconds;
    slew = new SlewRateLimiter(ratePerSec); // symmetric
  }

  public double getRampSeconds() { return rampSeconds; }

  /** Push ramp to both Spark MAXes using the new config API. */
  private void applyRampToSparks() {
    try {
      SparkMaxConfig cfg = new SparkMaxConfig();
      cfg.openLoopRampRate(rampSeconds);
      // If you later drive the Sparks in closed-loop, also set:
      // cfg.closedLoopRampRate(rampSeconds);

      // Apply to both motors (blocking apply; pass nulls for default timeout/persist)
      leftElevator.configure(cfg, null, null);
      rightElevator.configure(cfg, null, null);
    } catch (Exception e) {
      System.err.println("[Elevator] Failed to apply ramp config: " + e.getMessage());
    }

  }
  public boolean atGoal() {
    // true when PID is enabled and within tolerance
    return closedLoopEnabled && pid.atGoal();
  }
  

  // ---- Main loop ----
  @Override
  public void periodic() {


    
    if (!closedLoopEnabled) return; // manual mode leaves motors alone

    double output = pid.calculate(getPosition());
    output = MathUtil.clamp(output, -1.0, 1.0);
    setPercent(output); // retains soft limits + ramp
  }
}
