package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.components.Intake;
import frc.robot.components.Drivetrain;
import frc.robot.components.Wrist;
import frc.robot.components.ManualElevator;
// import frc.robot.components.ProfiledElevator;
import frc.robot.auto.Forward;
// import frc.robot.auto.CenterSwitch;

public class Robot extends TimedRobot {
    private Drivetrain drivetrain;
    private Joystick driverJoystick;
    private Joystick operatorJoystick;
    private JoystickButton xboxB;
    private JoystickButton xboxX;
    private JoystickButton xboxA;
    private ManualElevator elevator;
    // private ProfiledElevator elevator;
    private Intake intake;
    private Wrist wrist;

    private SendableChooser<AutoMode> autoChooser;
    private Forward autoForward;
    // private CenterSwitch autoCenterSwitch;

    private enum AutoMode {
        NONE, FORWARD// , CENTER_SWITCH
    }

    public void robotInit() {
        WPI_TalonSRX leftDrive = new WPI_TalonSRX(0);
        WPI_TalonSRX rightDrive = new WPI_TalonSRX(2);

        WPI_TalonSRX leftFollower = new WPI_TalonSRX(1);
        leftFollower.set(ControlMode.Follower, leftDrive.getDeviceID());

        WPI_TalonSRX rightFollower = new WPI_TalonSRX(3);
        rightFollower.set(ControlMode.Follower, rightDrive.getDeviceID());

        WPI_VictorSPX rightIntakeMotor = new WPI_VictorSPX(4);
        WPI_VictorSPX leftIntakeMotor = new WPI_VictorSPX(5);

        DoubleSolenoid intakeSolenoid = new DoubleSolenoid(2, 3);

        WPI_TalonSRX winch = new WPI_TalonSRX(6);
        DigitalInput bottomLimit = new DigitalInput(0);

        AHRS navx = new AHRS(Port.kMXP);

        WPI_TalonSRX wristMotor = new WPI_TalonSRX(8);

        drivetrain = new Drivetrain(leftDrive, rightDrive, navx);
        elevator = new ManualElevator(winch, bottomLimit);
        // elevator = new ProfiledElevator(winch, bottomLimit);
        intake = new Intake(leftIntakeMotor, rightIntakeMotor, intakeSolenoid);
        wrist = new Wrist(wristMotor);

        driverJoystick = new Joystick(0);
        operatorJoystick = new Joystick(1);

        xboxB = new JoystickButton(operatorJoystick, 2);
        xboxX = new JoystickButton(operatorJoystick, 3);
        xboxA = new JoystickButton(operatorJoystick, 4);

        autoChooser = new SendableChooser<AutoMode>();
        autoChooser.addDefault("Drive Forward", AutoMode.FORWARD);
        autoChooser.addObject("None", AutoMode.NONE);
        // autoChooser.addObject("Center Switch", AutoMode.CENTER_SWITCH);
        SmartDashboard.putData("Autonomous mode chooser", autoChooser);
        autoForward = new Forward(drivetrain);
        // autoCenterSwitch = new CenterSwitch(drivetrain, elevator, intake);

        setPeriod(0.02);
    }

    public void autonomousInit() {
        wrist.resetTop();
    }

    public void teleopPeriodic() {
        wrist.setTarget(Wrist.Target.UP);
        if (xboxB.get()) {
            intake.intake();
            wrist.setTarget(Wrist.Target.DOWN);
        } else if (xboxX.get()) {
            intake.outtake();
        }

        if (xboxA.get()) {
            wrist.setTarget(Wrist.Target.DOWN);
        }

        elevator.setSpeed(operatorJoystick.getY());

        wrist.update();
        intake.update();
        elevator.update();
        drivetrain.arcadeDrive(-driverJoystick.getY(), driverJoystick.getX());
    }

    public void autonomousPeriodic() {
        AutoMode autoMode = autoChooser.getSelected();
        if (autoMode == AutoMode.NONE) {
            return;
        }

        // if (autoMode == AutoMode.CENTER_SWITCH) {
        // autoCenterSwitch.update();
        // }

        if (autoMode == AutoMode.FORWARD) {
            autoForward.update();
        }
    }
}
