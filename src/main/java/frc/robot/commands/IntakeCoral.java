/*package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Intake;

public class IntakeCoral extends Command {
    public final Intake intakeSubsystem;
    public final double speed;

    //Creates new Feed
    public IntakeCoral(Intake intakeSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;

        addRequirements(intakeSubsystem);
    }

    //Called everytime the scheduler runs while the command is scheduled
    @Override
    public void execute() {
        intakeSubsystem.set(speed);
    }

    //Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.set(0);
    }

    //Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}*/
