package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.teamcode.utilities.Component;
import org.firstinspires.ftc.teamcode.utilities.RobotConstants;

public class FlapsComponentV1 extends Component {
    private SimpleServo leftServo, rightServo;
    private ToggleButtonReader flapsReader;

    @Override
    public void initializeComponent() {
        leftServo = new SimpleServo(hardwareMap, RobotConstants.FLAPS_LEFT, 0, 180);
        rightServo = new SimpleServo(hardwareMap, RobotConstants.FLAPS_RIGHT, 0, 180);

        flapsReader = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.B);

        setServos(true);
    }

    @Override
    public void runLoop() {
        flapsReader.readValue();
        if (flapsReader.getState()) {
            setServos(false);
        } else {
            setServos(true);
        }
    }

    private void setServos(boolean up) {
        if (up) {
            leftServo.setPosition(0);
            rightServo.setPosition(1);
        } else {
            leftServo.setPosition(0.55);
            rightServo.setPosition(0.625);
        }
    }
}
