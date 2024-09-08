package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.teamcode.utilities.Component;
import org.firstinspires.ftc.teamcode.utilities.RobotConstants;

public class BasketComponentV1 extends Component {
    private SimpleServo leftServo, rightServo;
    private ButtonReader leftReader, rightReader;

    @Override
    public void initializeComponent() {
        leftServo = new SimpleServo(hardwareMap, RobotConstants.BASKET_LEFT_SERVO, 0, 180);
        rightServo = new SimpleServo(hardwareMap, RobotConstants.BASKET_RIGHT_SERVO, 0, 180);

        leftReader = new ButtonReader(ballerGamepad, GamepadKeys.Button.LEFT_BUMPER);
        rightReader = new ButtonReader(ballerGamepad, GamepadKeys.Button.RIGHT_BUMPER);

        leftServo.setPosition(0);
        rightServo.setPosition(1);
    }

    @Override
    public void runLoop() {
        if (ballerGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.15) {
            leftServo.setPosition(ballerGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        } else {
            leftServo.setPosition(leftReader.isDown() ? 0.9 : 0);
        }

        if (ballerGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.15) {
            rightServo.setPosition(1 - ballerGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        } else {
            rightServo.setPosition(rightReader.isDown() ? 0.1 : 1);
        }
    }
}
