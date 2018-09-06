package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.components.Drivetrain;
import frc.robot.components.ProfiledElevator;
import frc.robot.components.Intake;

public class CenterSwitch {
    private Drivetrain drivetrain;
    private Drivetrain.ProfileTask task;

    private enum State {
        FORWARD, TURN, TO_SWITCH, EJECT, REVERSE, DONE
    };

    private State state;
    private State prevState;
    private ProfiledElevator elevator;
    private Intake intake;
    private double stateStartTime;

    public CenterSwitch(Drivetrain drivetrain, ProfiledElevator elevator, Intake intake) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.intake = intake;
        this.state = State.FORWARD;

        stateStartTime = 0.0;

        // Set drive forward task - how far to drive forward?
        task = drivetrain.forwardTask(0.0);
    }

    public void update() {
        if (state == State.FORWARD) {
            if (task.update()) {
                // Go to turn state
                state = State.TURN;
                task = drivetrain.rotateTask(45);
            }
        } else if (state == State.TURN) {
            elevator.setTarget(ProfiledElevator.Target.SWITCH);
            if (task.update()) {
                // Go to switch state
                state = State.TO_SWITCH;
                task = drivetrain.rotateTask(45);
            }
        } else if (state == State.TO_SWITCH) {
            if (task.update()) {
                // Go to eject state
                state = State.EJECT;
                stateStartTime = Timer.getFPGATimestamp();
            }
        } else if (state == State.EJECT) {
            intake.outtake();
            double stateLength = 3.0;
            if (Timer.getFPGATimestamp() - stateStartTime > stateLength) {
                state = State.REVERSE;
                task = drivetrain.forwardTask(-2);
            }
        } else if (state == State.REVERSE) {
            if (task.update()) {
                // Go to end state
                state = State.DONE;
            }
        } else {
            drivetrain.arcadeDrive(0.0, 0.0);
        }
    }
}
