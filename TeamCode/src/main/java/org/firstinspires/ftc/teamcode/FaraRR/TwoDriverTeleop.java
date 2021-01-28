package org.firstinspires.ftc.teamcode.FaraRR;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="TwoDriver", group = "Demo")
public class TwoDriverTeleop extends OpMode {

    HardwareConfig hw;
    //OdometryConfig odometry;

    private final double COEFF_DRIVE_HIGH = 0.7;
    private final double COEFF_DRIVE_LOW = 0.3;

    private double coeff = COEFF_DRIVE_HIGH;

    private final double INTAKE_POWER = 0.7;
    private final double BRAT_POWER = 0.5;
    private final double LANSAT_POWER = 0.55;

    @Override
    public void init() {
        hw = new HardwareConfig(hardwareMap);
        //odometry = new OdometryConfig(hardwareMap);
    }

    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if(gamepad1.b)
            coeff = COEFF_DRIVE_HIGH;
        if(gamepad1.x)
            coeff = COEFF_DRIVE_LOW;

        double LF = hw.clipPower(drive + strafe + rotate) * coeff;
        double LB = hw.clipPower(drive - strafe + rotate) * coeff;
        double RF = hw.clipPower(drive - strafe - rotate) * coeff;
        double RB = hw.clipPower(drive + strafe - rotate) * coeff;

        hw.leftFront.setPower(LF);
        hw.leftBack.setPower(LB);
        hw.rightFront.setPower(RF);
        hw.rightBack.setPower(RB);

        ////////////////////// Driver 2
        if(gamepad2.right_trigger > 0.1)
            hw.intake.setPower(INTAKE_POWER * gamepad2.right_trigger);
        else if(gamepad2.left_trigger > 0.1)
            hw.intake.setPower(-INTAKE_POWER * gamepad2.left_trigger);
        else
            hw.intake.setPower(0);

        if(gamepad2.b)
            hw.brat.setPower(BRAT_POWER);
        else if(gamepad2.x)
            hw.brat.setPower(-BRAT_POWER);
        else
            hw.brat.setPower(0);

        if(gamepad2.dpad_right)
            hw.apucat.setPosition(0.2);
        else if(gamepad2.dpad_left)
            hw.apucat.setPosition(0.5);

        if(gamepad2.right_bumper)
            hw.lansat.setPower(LANSAT_POWER);
        else
            hw.lansat.setPower(0);

        telemetry.addData("bratPosition", hw.brat.getCurrentPosition());
        /*telemetry.addData("odoLeft", odometry.odoLeft.getCurrentPosition());
        telemetry.addData("odoCenter", odometry.odoCenter.getCurrentPosition());
        telemetry.addData("odoRight", odometry.odoRight.getCurrentPosition());*/
    }
}
