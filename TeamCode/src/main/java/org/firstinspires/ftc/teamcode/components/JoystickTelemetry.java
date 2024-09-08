package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.Component;

public class JoystickTelemetry extends Component {
    @Override
    public void initializeComponent() {

    }

    @Override
    public void runLoop() {
        telemetry.addLine("---------Driver-Joystick---------");
        telemetry.addData("Left X", driverGamepad.getLeftX());
        telemetry.addData("Left Y", driverGamepad.getLeftY());
        telemetry.addData("Right X", driverGamepad.getLeftX());
        telemetry.addData("Right Y", driverGamepad.getLeftY());
        telemetry.addLine("---------Baller-Joystick---------");
        telemetry.addData("Left X", ballerGamepad.getLeftX());
        telemetry.addData("Left Y", ballerGamepad.getLeftY());
        telemetry.addData("Right X", ballerGamepad.getLeftX());
        telemetry.addData("Right Y", ballerGamepad.getLeftY());
    }
}
