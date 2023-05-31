/*
 * Copyright (c) 2022.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package org.firstinspires.ftc.teamcode.auton;
import static me.wobblyyyy.pathfinder2.Pathfinder.newSimulatedPathfinder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import me.wobblyyyy.pathfinder2.Pathfinder;
import me.wobblyyyy.pathfinder2.drive.MecanumDrive;
import me.wobblyyyy.pathfinder2.geometry.Angle;
import me.wobblyyyy.pathfinder2.geometry.PointXYZ;
import me.wobblyyyy.pathfinder2.geometry.Translation;
import me.wobblyyyy.pathfinder2.robot.Robot;
import me.wobblyyyy.pathfinder2.robot.components.Motor;
import me.wobblyyyy.pathfinder2.robot.simulated.SimulatedRobot;

@Autonomous
public class ExampleFieldNavigation extends LinearOpMode {
    private Robot robot = new SimulatedRobot();
    private double coefficient = -0.05;
    private Pathfinder pathfinder = newSimulatedPathfinder(coefficient);
    private Motor fl = new Motor() {
        @Override
        public double getPower() {
            return 0.3;
        }

        @Override
        public void setPower(double power) {

        }
    };
    private Motor fr = new Motor() {
        @Override
        public double getPower() {
            return 0.3;
        }

        @Override
        public void setPower(double power) {

        }
    };
    private Motor bl = new Motor() {
        @Override
        public double getPower() {
            return 0.3;
        }

        @Override
        public void setPower(double power) {

        }
    };
    private Motor br = new Motor() {
        @Override
        public double getPower() {
            return 0.3;
        }

        @Override
        public void setPower(double power) {

        }
    };
    private DcMotor fll;
    private DcMotor frr;
    private DcMotor bll;
    private DcMotor brr;
    private MecanumDrive mecDrive;
    public void runOpMode() {
        frr = hardwareMap.get(DcMotor.class, "front_right");
        bll = hardwareMap.get(DcMotor.class, "back_left");
        brr = hardwareMap.get(DcMotor.class, "back_right");
        fll = hardwareMap.get(DcMotor.class, "front_left");
        mecDrive = new MecanumDrive(fl, fr, bl, br);
        while (!isStarted() && !isStopRequested()) {
//            fr.setDirection(DcMotorSimple.Direction.REVERSE);
//            bl.setDirection(DcMotorSimple.Direction.REVERSE);

            bll.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bll.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fll.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fll.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            brr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            brr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (opModeIsActive()) {
                mecDrive.setTranslation(new Translation(0, 1));
                bll.setPower(bl.getPower());
                brr.setPower(br.getPower());
                fll.setPower(fl.getPower());
                frr.setPower(fr.getPower());
               pathfinder.goTo(new PointXYZ(0, 0, Angle.fromDeg(0)));
               pathfinder.tickUntil();

               pathfinder.goTo(new PointXYZ(10, 0, Angle.fromDeg(0)));
               pathfinder.tickUntil();

               pathfinder.goTo(new PointXYZ(10, 10, Angle.fromDeg(0)));
               pathfinder.tickUntil();

               pathfinder.goTo(new PointXYZ(0, 10, Angle.fromDeg(0)));
               pathfinder.tickUntil();

               pathfinder.goTo(new PointXYZ(0, 0, Angle.fromDeg(0)));
               pathfinder.tickUntil();
            }
        }
    }
}