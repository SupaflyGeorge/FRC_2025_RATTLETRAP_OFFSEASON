package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
    //motors
    private final SparkMax algaeIntake = new SparkMax(12, MotorType.kBrushless);

    //Creates new Intake
    public AlgaeIntake() {}
    public void set(double speed) {
        algaeIntake.set(speed);
        
    }
}
