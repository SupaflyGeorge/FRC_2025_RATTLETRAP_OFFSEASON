package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Coral intake driven by CTRE Talon FXS (Phoenix 6). NOTE: FXS drives BRUSHED motors. */
public class CoralIntake extends SubsystemBase {
  // === set your real CAN id and bus name (your Tuner showed "rio") ===
  public static final int    CAN_ID  = 21;
  public static final String CAN_BUS = "rio";

  private final TalonFXS motor;
  private final DutyCycleOut dutyCtl = new DutyCycleOut(0); // percent [-1..+1]
  private final VoltageOut   voltCtl = new VoltageOut(0);   // volts

  public CoralIntake() {
    // create on explicit bus, fall back to default if needed
    TalonFXS m;
    try { m = new TalonFXS(CAN_ID, CAN_BUS); } catch (Throwable t) { m = new TalonFXS(CAN_ID); }
    motor = m;

    // ---- enforce correct commutation for a 2-lead brushed motor on A+/A− ----
    TalonFXSConfiguration cfg = new TalonFXSConfiguration();
    cfg.Commutation.MotorArrangement   = MotorArrangementValue.Minion_JST;     // use A bridge only
    cfg.Commutation.BrushedMotorWiring = BrushedMotorWiringValue.Leads_A_and_B;     // two leads on A+/A−
    // (leave Advanced Hall Support disabled)
    motor.getConfigurator().apply(cfg);

    // neutral mode + clear faults
    motor.setNeutralMode(NeutralModeValue.Brake);
    motor.clearStickyFaults();

    // optional: invert if needed
    // motor.setInverted(true);

    // nice-to-have: faster status rates for diag (non-blocking)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, // Hz
        motor.getDutyCycle(),
        motor.getMotorVoltage(),
        motor.getSupplyCurrent(),
        motor.getStatorCurrent(),
        motor.getFaultField()
    );
  }

  // ---------- controls ----------
  /** Percent output [-1..+1]. */
  public void setPercent(double percent) {
    motor.clearStickyFaults();
    motor.setControl(dutyCtl.withOutput(percent));
  }

  /** Voltage output (typ. -12..+12 V). */
  public void setVolts(double volts) {
    motor.clearStickyFaults();
    motor.setControl(voltCtl.withOutput(volts));
  }

  public void stop() { setPercent(0); }

  // ---------- diagnostics helpers ----------
  public double getAppliedDuty()  { return motor.getDutyCycle().getValueAsDouble(); }
  public double getMotorVoltage() { return motor.getMotorVoltage().getValueAsDouble(); }
  public double getMeasuredCurrent() {
    try { return motor.getSupplyCurrent().getValueAsDouble(); }
    catch (Throwable t) { return motor.getStatorCurrent().getValueAsDouble(); }
  }
  public long getFaultBits() { return motor.getFaultField().getValue(); }
}

