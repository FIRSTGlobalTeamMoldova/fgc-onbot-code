package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name = "Trajectory Test")
public class TrajectoryTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(new Translation2d(), new Rotation2d());
        Pose2d endPose = new Pose2d(new Translation2d(1, 1), new Rotation2d());

        ArrayList<Translation2d> interiorWaypoints = new ArrayList<>();

        TrajectoryConfig config = new TrajectoryConfig(0.3, 0.3);

        double trackWidthMeters = 0.365;
        RamseteController controller = new RamseteController(0, 0);
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackWidthMeters);

        config.addConstraint(new DifferentialDriveKinematicsConstraint(kinematics, 0.3));

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPose, interiorWaypoints, endPose, config);
        ElapsedTime time = new ElapsedTime();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Trajectory.State currentState = trajectory.sample(time.seconds());
                ChassisSpeeds speeds = controller.calculate(currentState.poseMeters, currentState);
                DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
                double left = wheelSpeeds.leftMetersPerSecond;
                double right = wheelSpeeds.rightMetersPerSecond;

                // TODO: Convert meters per second to motor output (i don't know how 😢)

                telemetry.update();
                sleep(20);
            }
        }
    }
}
