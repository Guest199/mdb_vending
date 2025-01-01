package com.adytech.mdb_vending;

import java.util.HashMap;
import java.util.Map;

public enum Vending_State {
    VEND_STATE_IDLE(0),
    VEND_STATE_WAITING_SELECTION(1),
    VEND_STATE_WAITING_APPROVAL(2),
    VEND_STATE_SUPPLYING(3),
    VEND_STATE_FAILED(4),
    VEND_STATE_DENIED(5),
    VEND_STATE_SUCCEEDED(6);

    public final int code;

    private Vending_State(int code) {
        this.code = code;
    }

    private static final Map<Integer, Vending_State> by_code = new HashMap<>();
    static {
        for (Vending_State c: values()) {
            by_code.put(c.code, c);
        }
    }

    public static Vending_State by_code(int code)
    {
        return by_code.get(code);
    }
}
