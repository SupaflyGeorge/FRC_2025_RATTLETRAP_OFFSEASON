package frc.robot.subsystems.mechanisms;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CoralPivot extends SubsystemBase {
  private final SparkMax pivot = new SparkMax(14, MotorType.kBrushless);
  private final RelativeEncoder enc = pivot.getEncoder();

  public CoralPivot() {
    // Basic setup
    enc.setPosition(0.0);
    // Optionally set idle mode:
    // pivot.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
  }

  /** +1..-1 percent output */
  public void set(double percent) { pivot.set(percent); }
  public void stop() { set(0); }

  /** Position in motor rotations (adjust/scale if you add gearing) */
  public double getPosition() { return enc.getPosition(); }
  public double getVelocity() { return enc.getVelocity(); }
  public void resetEncoders() { enc.setPosition(0.0); }
}
