package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utilities.ComponentConfig;
import org.firstinspires.ftc.teamcode.utilities.Component;
import org.firstinspires.ftc.teamcode.utilities.RobotConstants;

import java.util.HashSet;

public class LinearMotionComponentV2 extends Component {
    private static class LinearMotionPosition {
        public int position;
        public double positionCoefficient;
        public ButtonReader buttonReader;

        public LinearMotionPosition(int position, GamepadEx gamepad, GamepadKeys.Button button, double positionCoefficient) {
            this.position = position;
            this.positionCoefficient = positionCoefficient;

            buttonReader = new ButtonReader(gamepad, button);
        }
    }

    private final HashSet<LinearMotionPosition> linearMotionPositions = new HashSet<>();
    private ToggleButtonReader boundsToggle;
    private ButtonReader encoderReset;
    private GravityGainMotor motionLeader;
    private MotorGroup linearMotion;

    boolean linearMotionIsGoingToPos;
    final double linearMotionBoundMin = 0,
            linearMotionBoundMax = 4300,
            linearMotionBoundSafetyBorder = 100;

    @Override
    public void initializeComponent() {
        motionLeader = new GravityGainMotor(hardwareMap, RobotConstants.LINEAR_LEFT);
        Motor follower = new Motor(hardwareMap, RobotConstants.LINEAR_RIGHT);
        follower.setInverted(true);

        linearMotion = new MotorGroup(motionLeader, follower);
        linearMotion.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        linearMotion.setPositionTolerance(10);
        linearMotion.resetEncoder();
        linearMotion.stopMotor();

        // Position configuration
        linearMotionPositions.add(new LinearMotionPosition(600, ballerGamepad, GamepadKeys.Button.A, 0.01));
        linearMotionPositions.add(new LinearMotionPosition(2700, ballerGamepad, GamepadKeys.Button.X, 0.05));
        linearMotionPositions.add(new LinearMotionPosition(3500, ballerGamepad, GamepadKeys.Button.B, 0.1));
        linearMotionPositions.add(new LinearMotionPosition(4330, ballerGamepad, GamepadKeys.Button.Y, 0.1));

        boundsToggle = new ToggleButtonReader(ballerGamepad, GamepadKeys.Button.DPAD_UP);
        encoderReset = new ButtonReader(ballerGamepad, GamepadKeys.Button.DPAD_LEFT);
    }

    @Override
    public void runLoop() {
        encoderCheck();
        telemetry();

        motionLeader.setGravityGain(linearMotion.getPositions().get(0) < 1700 ? 0 : 0.2);

        for (LinearMotionPosition pos : linearMotionPositions) {
            pos.buttonReader.readValue();
            if (pos.buttonReader.wasJustPressed()) {
                linearMotion.setRunMode(Motor.RunMode.PositionControl);
                linearMotion.setTargetPosition(pos.position);
                linearMotion.setPositionCoefficient(pos.positionCoefficient);
                linearMotionIsGoingToPos = true;
                ballerGamepad.gamepad.stopRumble();
            }
        }

        if (linearMotionIsGoingToPos) {
            if (!linearMotion.atTargetPosition()) {
                linearMotion.set(0.1);
            } else {
                linearMotion.set(0);
                linearMotionIsGoingToPos = false;
            }
        }

        double inputValue = ballerGamepad.getLeftY();
        if (Math.abs(ballerGamepad.getRightY()) > 0.05) {
            inputValue = -ballerGamepad.getRightY() / 2;
        }

        // If moving the joystick
        if (!linearMotionIsGoingToPos || inputValue != 0) {
            linearMotion.setRunMode(Motor.RunMode.RawPower);

            boundsToggle.readValue();
            if (!boundsToggle.getState()) {
                // Bounds logic
                double lmPos = linearMotion.getPositions().get(0);
                if (inputValue > 0 || (inputValue < 0 && lmPos > linearMotionBoundMin + linearMotionBoundSafetyBorder)) {
                    linearMotion.set(inputValue);
                    ballerGamepad.gamepad.stopRumble();
                } else if (inputValue != 0) {
                    linearMotion.set(0);
                    ballerGamepad.gamepad.rumble(100);
                } else {
                    linearMotion.set(0);
                    ballerGamepad.gamepad.stopRumble();
                }
            } else {
                linearMotion.set(inputValue);
                ballerGamepad.gamepad.stopRumble();
            }

            linearMotionIsGoingToPos = false;
        }
    }

    private void encoderCheck() {
        encoderReset.readValue();
        if (encoderReset.wasJustPressed()) {
            linearMotion.resetEncoder();
        }
    }

    private void telemetry() {
        telemetry.addLine();
        telemetry.addLine("Linear Motion Telemetry:");
        telemetry.addData("Linear motion pos", linearMotion.getPositions().get(0));
    }
}
