package frc.robot.components;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.motion.StaticProfile;
import frc.robot.pidf.Gains;
import frc.robot.pidf.PIDF;
import frc.robot.utils.Bounds;
//import frc.robot.utils.Plot;

public class Drivetrain {
    private enum ProfileTaskType {
        TURN, DRIVE
    }

    public class ProfileTask {
        private StaticProfile profile;
        private PIDF pidController;
        private double profileStartTime;
        private ProfileTaskType type;
        private double startValue, targetEndValue;

        private ProfileTask(StaticProfile profile, PIDF pidController, double profileStartTime, ProfileTaskType type) {
            this.profile = profile;
            this.pidController = pidController;
            this.type = type;
            this.profileStartTime = profileStartTime;

            // Plot profilePlot = new Plot("profile", profile::getVelocity,
            // profile.getDuration(), "Velocity", 0.025);
            // profilePlot.addSeries(profile::getPosition, "Position", 0.025);
            // profilePlot.addSeries(profile::getAcceleration, "Acceleration", 0.025);

            // profilePlot.savePlot("./graphs/auto/");

            targetEndValue = this.profile.getPosition(this.profile.getDuration());

            if (type == ProfileTaskType.DRIVE) {
                startValue = getTotalDistance();
                pidController.initialize(startValue, 0.0, 0.0);
            } else {
                startValue = getOrientation();
                pidController.initialize(0.0, 0.0, 0.0);
            }
        }

        public boolean update() {
            double time = Timer.getFPGATimestamp();
            double elapsedTime = time - profileStartTime;
            double currentTarget = profile.getPosition(elapsedTime);
            if (type == ProfileTaskType.DRIVE) {
                double targetVelocity = profile.getVelocity(elapsedTime);
                double speed = pidController.calculateOutput(getTotalDistance(), currentTarget, targetVelocity, 0.0,
                        elapsedTime);
                System.out.println("Target pos: " + (currentTarget - startValue) + "  Current pos: "
                        + (getTotalDistance() - startValue) + "  Target velo: " + targetVelocity + "  Output speed: "
                        + speed);
                arcadeDrive(speed, 0.0);
            } else {
                double rotation = pidController.calculateOutput(getOrientation() - startValue, currentTarget, 0.0, 0.0,
                        elapsedTime);
                arcadeDrive(0.0, rotation);
            }
            return (elapsedTime > profile.getDuration() && (targetEndValue - currentTarget) < (targetEndValue * 0.01));
        }
    }

    private WPI_TalonSRX rightDrive;
    private DifferentialDrive drive;
    private AHRS navx;

    public Drivetrain(WPI_TalonSRX leftDrive, WPI_TalonSRX rightDrive, AHRS navx) {
        leftDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);
        rightDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);
        drive = new DifferentialDrive(leftDrive, rightDrive);
        this.rightDrive = rightDrive;
        this.navx = navx;

        setOrientation(0.0);
    }

    public void arcadeDrive(double forward, double rotation) {
        drive.arcadeDrive(forward, rotation);
    }

    public ProfileTask forwardTask(double meters) {
        double currentDistance = getTotalDistance();
        StaticProfile profile = new StaticProfile(getVelocity(), currentDistance, currentDistance + meters, 0.5, 0.5,
                1.0);
        Gains gains = new Gains(0.5, 0.0, 0.0, 0.0, 0.0, 0.0);
        Bounds outputBounds = new Bounds(-0.8, 0.8);
        PIDF pidController = new PIDF(gains, outputBounds);
        double time = Timer.getFPGATimestamp();
        return new ProfileTask(profile, pidController, time, ProfileTaskType.DRIVE);
    }

    public ProfileTask rotateTask(double degrees) {
        double radians = Math.toRadians(degrees);
        double currentAngle = getOrientation();
        StaticProfile profile = new StaticProfile(getVelocity(), currentAngle, currentAngle + radians, 1.5, 0.75, 1.0);
        Gains gains = new Gains(0.85, 0.1, 0.1);
        Bounds outputBounds = new Bounds(-1.0, 1.0);
        PIDF pidController = new PIDF(gains, outputBounds);
        double time = Timer.getFPGATimestamp();
        return new ProfileTask(profile, pidController, time, ProfileTaskType.TURN);
    }

    public double getOrientation() {
        return Math.toRadians(-navx.getAngle());
    }

    public double getVelocity() {
        double revolutions = rightDrive.getSelectedSensorVelocity(0) / 4096.0;
        double velocity = revolutions * 6 * Math.PI * 0.02540;
        return velocity;
    }

    public double getTotalDistance() {
        double revolutions = rightDrive.getSelectedSensorPosition(0) / 4096.0;
        double distance = revolutions * 6 * Math.PI * 0.02540;
        return distance;
    }

    private void setOrientation(double orientation) {
        navx.setAngleAdjustment(0.0);
        navx.setAngleAdjustment(-navx.getAngle() - Math.toDegrees(orientation));
    }
}
