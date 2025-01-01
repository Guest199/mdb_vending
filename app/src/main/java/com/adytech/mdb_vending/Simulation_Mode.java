package com.adytech.mdb_vending;

import java.util.HashMap;
import java.util.Map;

public enum Simulation_Mode {
    NO_SIMULATION(0),
    REGULAR_SEQUENCE_SCENARIO(1),
    FAILURE_IN_SUPPLYING_SCENARIO(2),
    NO_SELECTION_SCENARIO(3);

    public final int code;

    private Simulation_Mode(int code) {
        this.code = code;
    }

    private static final Map<Integer, Simulation_Mode> by_code = new HashMap<>();
    static {
        for (Simulation_Mode c: values()) {
            by_code.put(c.code, c);
        }
    }

    public static Simulation_Mode by_code(int code)
    {
        return by_code.get(code);
    }
}
