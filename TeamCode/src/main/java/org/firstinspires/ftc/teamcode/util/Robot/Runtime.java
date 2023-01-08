package org.firstinspires.ftc.teamcode.util.Robot;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Runtime {
    private final ElapsedTime runtime;

    public Runtime() {
        runtime = new ElapsedTime();
    }

    public void reset() {
        runtime.reset();
    }

    public void wait(int time) {
        reset();
        while(runtime.milliseconds() < time) { }
    }

    public int getTime() {
        return (int) runtime.milliseconds();
    }
}