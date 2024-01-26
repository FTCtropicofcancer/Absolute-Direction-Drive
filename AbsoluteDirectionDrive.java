package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "AbsoluteDirectionDrive")
public class AbsoluteDirectionDrive<string> extends LinearOpMode {

    //set motor & servo
    private DcMotor LF;
    private DcMotor LB;
    private DcMotor RF;
    private DcMotor RB;
    BNO055IMU imu;

    @Override
    public void runOpMode() {

        //set variable
        double drive_o;
        double drive_y;
        double drive_x;
        double drive_speed = 0.7;
        double angle;
        double yLF;
        double yLB;
        double yRF;
        double yRB;
        double xLF;
        double xLB;
        double xRF;
        double xRB;

        //get motor & servo
        LF = hardwareMap.get(DcMotor.class, "LF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");

        //motor encoder set
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //set imu
        BNO055IMU.Parameters IMU_Parameters;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        IMU_Parameters = new BNO055IMU.Parameters();
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(IMU_Parameters);

        waitForStart();

        if (opModeIsActive()) {

            //set motor reverse
            RF.setDirection(DcMotorSimple.Direction.REVERSE);
            LB.setDirection(DcMotorSimple.Direction.REVERSE);

            //set motor stop action
            LF.setZeroPowerBehavior(BRAKE);
            LB.setZeroPowerBehavior(BRAKE);
            RF.setZeroPowerBehavior(BRAKE);
            RB.setZeroPowerBehavior(BRAKE);

            while (opModeIsActive()) {

                //get gamepad information
                drive_o = gamepad1.right_stick_x;
                drive_y = gamepad1.left_stick_y;
                drive_x = gamepad1.left_stick_x;

                //get imu information
                angle = -(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle);

                //gamepad caculate
                double boxy=(double)gamepad1.left_stick_y;
                double boxx=(double)gamepad1.left_stick_x;
                double v = Math.abs(Math.sqrt(Math.pow(boxy, 2) + Math.pow(boxx, 2)));
                if(v <10){
                    drive_speed=(float)0;
                }else if(v <50){
                    drive_speed=(float)0.4;
                }else{
                    drive_speed= (float) 0.7;
                }
                if(gamepad1.right_stick_x>0&&gamepad1.right_stick_x<10){
                    drive_speed=(float)0;
                }else if(gamepad1.right_stick_x<50){
                    drive_speed=(float)0.4;
                }else{
                    drive_speed= (float) 0.7;
                }

                //imu angle calculate
                if (angle > 180){
                    angle = angle - 360;
                }
                if (angle < -180){
                    angle = angle + 360;
                }
                if (angle >= -45 && angle <= 135){
                    yLF = -(angle/45-1);
                    yRB = -(angle/45-1);
                    xLB = (angle/45)-1;
                    xRF = (angle/45)-1;
                } else if (angle > 0){
                    angle = angle - 360;
                    yLF = (angle/45)+3;
                    yRB = (angle/45)+3;
                    xLB = -(angle/45+3);
                    xRF = -(angle/45+3);
                } else {
                    yLF = (angle/45)+3;
                    yRB = (angle/45)+3;
                    xLB = -(angle/45+3);
                    xRF = -(angle/45+3);
                }
                if (angle >= -135 && angle <= 45){
                    yLB = (angle/45)+1;
                    yRF = (angle/45)+1;
                    xLF = (angle/45)+1;
                    xRB = (angle/45)+1;
                } else if (angle < 0){
                    angle = angle + 360;
                    yLB = -(angle/45-3);
                    yRF = -(angle/45-3);
                    xLF = -(angle/45-3);
                    xRB = -(angle/45-3);
                } else {
                    yLB = -(angle/45-3);
                    yRF = -(angle/45-3);
                    xLF = -(angle/45-3);
                    xRB = -(angle/45-3);
                }

                //wheels caculate
                RF.setPower( ( drive_y * yRF - drive_x * xRF + drive_o ) * drive_speed );
                RB.setPower( ( drive_y * yRB - drive_x * xRB + drive_o ) * drive_speed );
                LF.setPower( ( drive_y * yLF - drive_x * xLF - drive_o ) * drive_speed );
                LB.setPower( ( drive_y * yLB - drive_x * xLB - drive_o ) * drive_speed );

                //telemetry
                telemetry.addData("angle", angle);
                telemetry.addData("LF", LF.getCurrentPosition());
                telemetry.addData("LB", LB.getCurrentPosition());
                telemetry.addData("RF", RF.getCurrentPosition());
                telemetry.addData("RB", RB.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}