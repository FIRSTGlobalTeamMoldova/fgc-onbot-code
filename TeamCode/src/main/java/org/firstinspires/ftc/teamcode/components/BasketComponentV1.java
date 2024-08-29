package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utilities.IComponent;
import org.firstinspires.ftc.teamcode.utilities.RobotConstants;

public class BasketComponentV1 implements IComponent {
    private final GamepadEx targetGamepad;
    private final MotorGroup basket;

    public BasketComponentV1(GamepadEx targetGamepad, HardwareMap hardwareMap) {
        Motor left = new Motor(hardwareMap, RobotConstants.BASKET_LEFT);
        Motor right = new Motor(hardwareMap, RobotConstants.BASKET_RIGHT);
        right.setInverted(true);

        basket = new MotorGroup(left, right);

        this.targetGamepad = targetGamepad;
    }

    @Override
    public void runLoop() {
        basket.set(targetGamepad.getRightY());
    }
}
