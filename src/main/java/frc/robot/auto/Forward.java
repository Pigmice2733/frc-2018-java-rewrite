package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.drivetrain.Drivetrain;

public class Forward {
    private Drivetrain drivetrain;

    public Forward(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void update() {
        final double time = Timer.getFPGATimestamp();

        if (time < 3) {
            drivetrain.arcadeDrive(0.3, 0);
        }
    }
}
