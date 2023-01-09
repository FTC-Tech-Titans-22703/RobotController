package org.firstinspires.ftc.teamcode.util.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

public class TelemetryLogger {
    private final Telemetry telemetry;
    private final HashMap<String, Object> data;

    public TelemetryLogger(Telemetry telemetry) {
        this.telemetry = telemetry;
        data = new HashMap<>();
    }

    public void update(String caption, Object value) {
        update(caption, value, true);
    }

    public void update(String caption, Object value, boolean update) {
        data.put(caption, value);
        if(update) {
            for(Map.Entry<String, Object> element : data.entrySet()) {
                telemetry.addData(element.getKey(), element.getValue());
            }

            telemetry.update();
        }
    }

    public void remove(String caption) {
        data.remove(caption);
    }
}
