package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.BaseVelocityTelemetry;
import org.firstinspires.ftc.teamcode.components.BasketComponentV1;
import org.firstinspires.ftc.teamcode.components.FlapsComponentV1;
import org.firstinspires.ftc.teamcode.components.HDriveComponentV3;
import org.firstinspires.ftc.teamcode.components.JoystickTelemetry;
import org.firstinspires.ftc.teamcode.components.LinearMotionComponentV2;
import org.firstinspires.ftc.teamcode.utilities.ComponentOpMode;

@TeleOp(name = "Reliable")
public class RobotTestingV6 extends ComponentOpMode {
    @Override
    public void componentsSetup() {
        components.add(new HDriveComponentV3());
        components.add(new LinearMotionComponentV2());
        components.add(new BasketComponentV1());
        components.add(new FlapsComponentV1());

        components.add(new BaseVelocityTelemetry());
        components.add(new JoystickTelemetry());
    }
}
