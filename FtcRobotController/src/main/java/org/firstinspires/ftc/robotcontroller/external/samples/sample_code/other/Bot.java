package org.firstinspires.ftc.robotcontroller.external.samples.sample_code.other;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

public class Bot {
    private HardwareMap map;
    public Telemetry telemetry;

    public final DcMotor leftTop;
    public final DcMotor leftBottom;
    public final DcMotor rightTop;
    public final DcMotor rightBottom;
    public final DigitalChannel button;

    public Runtime runtime;
    public Printer console;
    public Gate1 gate1;
    public Gate2 gate2;

    public Bot(HardwareMap map, Telemetry telemetry) {
        this.map = map;
        this.telemetry = telemetry;

        leftTop = map.get(DcMotor.class, "leftTop");
        leftBottom = map.get(DcMotor.class, "leftBottom");
        rightTop = map.get(DcMotor.class, "rightTop");
        rightBottom = map.get(DcMotor.class, "rightBottom");

        button = map.get(DigitalChannel.class, "button");
        button.setMode(DigitalChannel.Mode.INPUT);

        runtime = new Runtime();
        console = new Printer();
        gate1 = new Gate1();
        gate2 = new Gate2();
    }

    public void init() {
        leftTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftTop.setTargetPosition(0);
        leftTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBottom.setTargetPosition(0);
        leftBottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightTop.setTargetPosition(0);
        rightTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBottom.setTargetPosition(0);
        rightBottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stopAndReset() {
        leftTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftTop.setTargetPosition(0);
        leftTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBottom.setTargetPosition(0);
        leftBottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTop.setTargetPosition(0);
        rightTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBottom.setTargetPosition(0);
        rightBottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public class Runtime {
        private ElapsedTime runtime;

        public Runtime() {
            runtime = new ElapsedTime();
        }

        public void reset() {
            runtime.reset();
        }

        public void delay(int millis) {
            try {
                Thread.sleep(millis);
            } catch (InterruptedException ignored) {
            }
        }

        public int getTime() {
            return (int)runtime.milliseconds();
        }

    }

    public class Gate1 {
        private boolean state;

        public Gate1() {
            state = false;
        }

        public void close() {
            state = false;
            console.update("Gate 1", this.getState(), true);
        }

        public void open() {
            state = true;
            console.update("Gate 1", this.getState(), true);
        }

        public boolean getState() {
            return state;
        }
    }

    public class Gate2 {
        private boolean state;

        public Gate2() {
            state = false;
        }

        public void close() {
            state = false;
            console.update("Gate 2", this.getState(), true);
        }

        public void open() {
            state = true;
            console.update("Gate 2", this.getState(), true);
        }

        public boolean getState() {
            return state;
        }
    }

    public class Printer {
        private HashMap<String, Object> data;

        public Printer() {
            data = new HashMap<String, Object>();
        }

        public void update(String caption, Object value, boolean update) {
            data.put(caption, value);
            if(update) {
                for(Map.Entry element : data.entrySet()) {
                    telemetry.addData(element.getKey().toString(), element.getValue());
                }
                telemetry.update();
            }
        }
    }
}

