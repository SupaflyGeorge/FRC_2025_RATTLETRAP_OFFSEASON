package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.CoralIntake;

/** Non-blocking, safe diagnostic: prints duty/volts/amps/faults while commanding %. */
public class CoralIntakeSafeDiag extends Command {
  private final CoralIntake intake;
  private final double percent;
  private int loops;

  public CoralIntakeSafeDiag(CoralIntake intake, double percent) {
    this.intake = intake;
    this.percent = percent;
    addRequirements(intake);
  }

  @Override public void initialize() {
    loops = 0;
    System.out.println("[IntakeSafeDiag] start, cmd=" + percent);
    intake.setPercent(percent);
  }

  @Override public void execute() {
    if (++loops % 10 != 0) return; // print ~every 200ms

    if (!RobotBase.isReal()) {
      System.out.println("[IntakeSafeDiag] (sim) cmd=" + percent);
      return;
    }

    double duty = Double.NaN, volts = Double.NaN, amps = Double.NaN;
    long faults = 0L;
    try { duty   = intake.getAppliedDuty();  } catch (Throwable ignored) {}
    try { volts  = intake.getMotorVoltage(); } catch (Throwable ignored) {}
    try { amps   = intake.getMeasuredCurrent(); } catch (Throwable ignored) {}
    try { faults = intake.getFaultBits(); } catch (Throwable ignored) {}

    System.out.printf("[IntakeSafeDiag] cmd=%.2f duty=%.3f V=%.1f I=%.1f faults=0x%X%n",
        percent, duty, volts, amps, faults);
  }

  @Override public void end(boolean interrupted) {
    intake.stop();
    System.out.println("[IntakeSafeDiag] end interrupted=" + interrupted);
  }

  @Override public boolean isFinished() { return false; }
}
