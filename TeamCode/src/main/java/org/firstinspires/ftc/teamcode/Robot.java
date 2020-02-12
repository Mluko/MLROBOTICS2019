package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot
{
    //   [1|2]   //  [0-3]  //   [Y|N]  //   [Y|N]
    // Motors                             name               // Rev Hub # //  Port # // Attached //  in hMap
    private static final String FRDrive = "frontRightDrive"; //    1      //    0    //     Y    //    Y
    private static final String FLDrive = "frontLeftDrive";  //    1      //    1    //     Y    //    Y
    private static final String BRDrive = "backRightDrive";  //    1      //    2    //     Y    //    Y
    private static final String BLDrive = "backLeftDrive";   //    1      //    3    //     Y    //    Y
    private static final String RCollect= "rightCollection"; //    2      //    0    //     Y    //    Y
    private static final String LCollect= "leftCollection";  //    2      //    1    //     Y    //    Y
    private static final String RLift   = "rightLift";       //    2      //    2    //     N    //    Y
    private static final String LLift   = "leftLift";        //    2      //    3    //     N    //    Y
    //  [1|2]    //  [0-5]  //   [Y|N]  //   [Y|N]
    // Servos                             name               // Rev Hub # //  Port # // Attached //  in hMap
    private static final String RClamp  = "rightClamp";      //    1      //    0    //     Y    //    Y
    private static final String LClamp  = "leftClamp";       //    1      //    1    //     Y    //    Y


    public DcMotor frontRightDrive, frontLeftDrive, backRightDrive, backLeftDrive;
    public DcMotor rightCollection, leftCollection;
    public DcMotor rightLift, leftLift;
    public Servo rightClamp, leftClamp;

    public static final double CLAMPSERVOINITPOS = -0.5;
    public static final double CLAMPSERVOUPPOS = -0.5;
    public static final double CLAMPSERVODOWNPOS = 0.9;
    public boolean clampIsUP;


    public Robot()
    { }

    public void init(HardwareMap hMap)
    {
        // map to hardware for drive motors;
        frontRightDrive = hMap.get(DcMotor.class, FRDrive);
        frontLeftDrive  = hMap.get(DcMotor.class, FLDrive);
        backRightDrive  = hMap.get(DcMotor.class, BRDrive);
        backLeftDrive   = hMap.get(DcMotor.class, BLDrive);

        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // set direction for drive motors;
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        // set Zero Power Mode for drive motors;
        frontRightDrive.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        frontLeftDrive.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        backLeftDrive.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );

        // map collection motors to hardware
        rightCollection = hMap.get(DcMotor.class, RCollect);
        leftCollection  = hMap.get(DcMotor.class, LCollect);

        // set direction of collection motors
        rightCollection.setDirection(DcMotor.Direction.FORWARD);
        leftCollection.setDirection(DcMotor.Direction.REVERSE);

        // map lift motors to hardware
        rightLift = hMap.get(DcMotor.class, RLift);
        leftLift = hMap.get(DcMotor.class, LLift);


        // set direction of lift motors
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setDirection(DcMotor.Direction.REVERSE);



        // map foundation clamping servos to hardware
        rightClamp = hMap.get(Servo.class, RClamp);
        leftClamp  = hMap.get(Servo.class, LClamp);

        // set direction of clamping servos
        rightClamp.setDirection(Servo.Direction.FORWARD);
        leftClamp.setDirection(Servo.Direction.REVERSE);
        //-----------------------------------------------------------------------------------------------
        initialPos();
    }

    /**
     * this method sets the inital posiion of all servos and motors on the robot
     */
    public void initialPos()
    {
        clamp(CLAMPSERVOINITPOS);
        clampIsUP = true;
        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        rightCollection.setPower(0);
        leftCollection.setPower(0);
    }


    public void controlButtonDrive(double power, boolean forward, boolean reverse, boolean sRight, boolean sLeft)
    {
        if(forward)
            this.drive(power);
        else if(reverse)
            this.drive(power);
        else if(sRight)
            this.strafe(power);
        else if(sLeft)
            this.strafe(power);
    }

    public void controlJoyStickDrive(double left_stick_x, double left_stick_y, double right_stick_x, double right_stick_y)
    {
        double magnitude = Math.hypot(left_stick_x, left_stick_y);
        double robotAngle = Math.atan2(left_stick_y, left_stick_x) - Math.PI / 4;
        double rightX = right_stick_x;

        final double fl = magnitude * Math.cos(robotAngle) + rightX;
        final double fr = magnitude * Math.sin(robotAngle) - rightX;
        final double bl = magnitude * Math.sin(robotAngle) + rightX;
        final double br = magnitude * Math.cos(robotAngle) - rightX;

        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }


    public void controlCollect(double power, boolean forward, boolean reverse)
    {
        if(forward)
            this.collect(power);
        else if(reverse)
            this.collect(-power);
        else
            this.collect(0);
    }

    public void controlLift(double power, boolean up, boolean down)
    {
        if(up)
            this.lift(power);
        else if(down)
            this.lift(-power);
        else
            this.collect(0);
    }

    public void controlClamp(boolean state) {
        if (state) {
            if (clampIsUP) {
                clamp(CLAMPSERVODOWNPOS);
            } else {
                clamp(CLAMPSERVOUPPOS);
            }
            clampIsUP = !clampIsUP;
        }
    }

    public void collect(double power)
    {
        rightCollection.setPower(power);
        leftCollection.setPower(power);
    }

    public void drive(double power)
    {
        frontRightDrive.setPower(power);
        frontLeftDrive.setPower(power);
        backRightDrive.setPower(power);
        backLeftDrive.setPower(power);
    }

    public void rightDrive(double power)
    {
        frontRightDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    public void leftDrive(double power)
    {
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(power);
    }

    public void lift(double power)
    {
        rightLift.setPower(power);
        leftLift.setPower(power);
    }

    public void strafe(double power)
    {
        frontRightDrive.setPower(-power);
        frontLeftDrive.setPower(power);
        backRightDrive.setPower(power);
        backLeftDrive.setPower(-power);
    }

    public void clamp(double position)
    {
        rightClamp.setPosition(position);
        leftClamp.setPosition(position);
    }


}


