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
        public ButtonReader buttonReader;

        public LinearMotionPosition(int position, GamepadEx gamepad, GamepadKeys.Button button) {
            this.position = position;

            buttonReader = new ButtonReader(gamepad, button);
        }
    }

    private final HashSet<LinearMotionPosition> linearMotionPositions = new HashSet<>();
    private ToggleButtonReader boundsToggle;
    private ButtonReader encoderReset;
    private MotorGroup linearMotion;

    boolean linearMotionIsGoingToPos;
    final double linearMotionBoundMin = 0,
            linearMotionBoundMax = 4300,
            linearMotionBoundSafetyBorder = 100;

    @Override
    public void initializeComponent() {
        GravityGainMotor leader = new GravityGainMotor(hardwareMap, RobotConstants.LINEAR_LEFT);
        leader.setGravityGain(0);
        Motor follower = new Motor(hardwareMap, RobotConstants.LINEAR_RIGHT);

        linearMotion = new MotorGroup(leader, follower);
        linearMotion.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        linearMotion.setPositionCoefficient(0.01);
        linearMotion.setPositionTolerance(10);
        linearMotion.setInverted(true);
        linearMotion.resetEncoder();
        linearMotion.stopMotor();

        // Position configuration
        linearMotionPositions.add(new LinearMotionPosition(0, ballerGamepad, GamepadKeys.Button.A));
        linearMotionPositions.add(new LinearMotionPosition(2500, ballerGamepad, GamepadKeys.Button.X));
        linearMotionPositions.add(new LinearMotionPosition(3500, ballerGamepad, GamepadKeys.Button.B));
        linearMotionPositions.add(new LinearMotionPosition(4300, ballerGamepad, GamepadKeys.Button.Y));

        boundsToggle = new ToggleButtonReader(ballerGamepad, GamepadKeys.Button.DPAD_UP);
        encoderReset = new ButtonReader(ballerGamepad, GamepadKeys.Button.DPAD_LEFT);
    }

    @Override
    public void runLoop() {
        encoderCheck();
        telemetry();

        for (LinearMotionPosition pos : linearMotionPositions) {
            pos.buttonReader.readValue();
            if (pos.buttonReader.wasJustPressed()) {
                linearMotion.setRunMode(Motor.RunMode.PositionControl);
                linearMotion.setTargetPosition(pos.position);
                linearMotionIsGoingToPos = true;
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

        // If not going towards a target or moving the joystick
        if (!linearMotionIsGoingToPos || ballerGamepad.getLeftY() != 0) {
            linearMotion.setRunMode(Motor.RunMode.RawPower);

            boundsToggle.readValue();
            if (!boundsToggle.getState()) {
                // Bounds logic
                double lmPos = linearMotion.getPositions().get(0);
                if ((ballerGamepad.getLeftY() > 0 && lmPos < linearMotionBoundMax - linearMotionBoundSafetyBorder) ||
                        (ballerGamepad.getLeftY() < 0 && lmPos > linearMotionBoundMin + linearMotionBoundSafetyBorder)) {
                    linearMotion.set(scaledJoystickDir());
                } else {
                    linearMotion.set(0);
                }
            } else {
                linearMotion.set(scaledJoystickDir());
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
        telemetry.addLine("Linear Motion Telemetry:");
        telemetry.addData("Linear motion pos", linearMotion.getPositions().get(0));
        telemetry.addLine("Linear Motion Config:");
        telemetry.addData("Bounds Toggle", "Dpad up");
        telemetry.addData("Reset Encoder", "Dpad left");
    }

    private double scaledJoystickDir() {
        if (ballerGamepad.getLeftY() < 0) {
            return ballerGamepad.getLeftY() / 4;
        } else {
            return ballerGamepad.getLeftY();
        }
    }
}
