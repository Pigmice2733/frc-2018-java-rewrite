package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.components.Drivetrain;

public class Forward {
    private Drivetrain drivetrain;
    private double startTime;

    public Forward(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.startTime = -1.0;
    }

    public void update() {
        if (startTime == -1.0) {
            startTime = Timer.getFPGATimestamp();
        }

        if (Timer.getFPGATimestamp() - startTime < 2) {
            drivetrain.arcadeDrive(0.5, 0);
        }
    }
}
