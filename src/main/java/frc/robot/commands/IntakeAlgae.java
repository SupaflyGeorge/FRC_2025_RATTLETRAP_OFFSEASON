package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.AlgaeIntake;

public class IntakeAlgae extends Command {
    public final AlgaeIntake algaeIntakeSubsystem;
    public final double speed;

    //Creates new Feed
    public IntakeAlgae (AlgaeIntake algaeIntakeSubsystem, double speed) {
        this.algaeIntakeSubsystem = algaeIntakeSubsystem;
        this.speed = speed;

        addRequirements(algaeIntakeSubsystem);
    }

    //Called everytime the scheduler runs while the command is scheduled
    @Override
    public void execute() {
        algaeIntakeSubsystem.set(speed);
    }

    //Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        algaeIntakeSubsystem.set(0);
    }

    //Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}
