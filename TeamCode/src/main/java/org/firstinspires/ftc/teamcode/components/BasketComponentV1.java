package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.teamcode.utilities.Component;
import org.firstinspires.ftc.teamcode.utilities.RobotConstants;

public class BasketComponentV1 extends Component {
    private MotorGroup basket;

    @Override
    public void initializeComponent() {
        Motor left = new Motor(hardwareMap, RobotConstants.BASKET_LEFT);
        Motor right = new Motor(hardwareMap, RobotConstants.BASKET_RIGHT);
        right.setInverted(true);

        basket = new MotorGroup(left, right);
    }

    @Override
    public void runLoop() {
        basket.set(ballerGamepad.getRightY());
    }
}
