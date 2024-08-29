package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utilities.IComponent;
import org.firstinspires.ftc.teamcode.utilities.RobotConstants;

import java.util.HashSet;

public class LinearMotionComponentV1 implements IComponent {
    private class LinearMotionPosition {
        public int position;
        public ButtonReader buttonReader;

        public LinearMotionPosition(int position, GamepadEx gamepad, GamepadKeys.Button button) {
            this.position = position;

            buttonReader = new ButtonReader(gamepad, button);
        }
    }

    private final GamepadEx targetGamepad;

    private final HashSet<LinearMotionPosition> linearMotionPositions = new HashSet<>();
    private final MotorGroup linearMotion;

    private boolean useBounds = true;

    public LinearMotionComponentV1(GamepadEx targetGamepad, HardwareMap hardwareMap) {
        Motor leader = new Motor(hardwareMap, RobotConstants.LINEAR_LEFT);
        Motor follower = new Motor(hardwareMap, RobotConstants.LINEAR_RIGHT);

        linearMotion = new MotorGroup(leader, follower);
        linearMotion.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        linearMotion.setPositionCoefficient(0.05);
        linearMotion.setPositionTolerance(10);
        linearMotion.setInverted(true);
        linearMotion.resetEncoder();
        linearMotion.stopMotor();

        // Position configuration
        linearMotionPositions.add(new LinearMotionPosition(0, targetGamepad, GamepadKeys.Button.A));
        linearMotionPositions.add(new LinearMotionPosition(4000, targetGamepad, GamepadKeys.Button.X));
        linearMotionPositions.add(new LinearMotionPosition(6000, targetGamepad, GamepadKeys.Button.B));
        linearMotionPositions.add(new LinearMotionPosition(8000, targetGamepad, GamepadKeys.Button.Y));

        this.targetGamepad = targetGamepad;
    }

    public LinearMotionComponentV1 IgnoreBounds() {
        useBounds = false;
        return this;
    }

    boolean linearMotionIsGoingToPos;
    final double linearMotionBoundMin = 0,
            linearMotionBoundMax = 8000,
            linearMotionBoundSafetyBorder = 100;

    @Override
    public void runLoop() {
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
                linearMotion.stopMotor();
                linearMotionIsGoingToPos = false;
            }
        }

        // If not going towards a target or moving the joystick
        if (!linearMotionIsGoingToPos || targetGamepad.getLeftY() != 0) {
            linearMotion.setRunMode(Motor.RunMode.RawPower);

            if (useBounds) {
                // Bounds logic
                double lmPos = linearMotion.getPositions().get(0);
                if ((targetGamepad.getLeftY() > 0 && lmPos < linearMotionBoundMax - linearMotionBoundSafetyBorder) ||
                        (targetGamepad.getLeftY() < 0 && lmPos > linearMotionBoundMin + linearMotionBoundSafetyBorder)) {
                    linearMotion.set(targetGamepad.getLeftY());
                } else {
                    linearMotion.stopMotor();
                }
            } else {
                linearMotion.set(targetGamepad.getLeftY());
            }

            linearMotionIsGoingToPos = false;
        }
    }
}
