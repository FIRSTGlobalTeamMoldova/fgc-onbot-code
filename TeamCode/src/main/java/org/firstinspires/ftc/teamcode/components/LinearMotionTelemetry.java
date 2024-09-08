package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.utilities.Component;
import org.firstinspires.ftc.teamcode.utilities.RobotConstants;

public class LinearMotionTelemetry extends Component {
    private Motor linearLeaderMotor;

    @Override
    public void initializeComponent() {
        linearLeaderMotor = new Motor(hardwareMap, RobotConstants.LINEAR_LEFT);
        linearLeaderMotor.resetEncoder();
    }

    @Override
    public void runLoop() {
        telemetry.addLine("---------Linear-Motion-Telemetry---------");
        telemetry.addData("Linear motion left pos", linearLeaderMotor.getCurrentPosition());
    }
}
