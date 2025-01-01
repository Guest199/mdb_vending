package com.adytech.mdb_vending;


/**
 * Main interface class to AbsEmv services
 */
public class MDB_Vending {

    public void MDB_start_driver(){
        System.loadLibrary("mdb_vending-lib");

        new Thread(new Runnable() {
            @Override
            public void run() {
                native_MDB_slave_func();
            }
        }).start();
    }

    public int MDB_get_max_price(){
        return native_MDB_get_max_price();
    }

    public void MDB_set_user_funds(int user_funds){
        native_MDB_set_user_funds(user_funds);
    }

    public void MDB_end_vend_state(){
        native_MDB_end_vend_state();
    }
    public void MDB_start_vending_state() {
        native_MDB_start_vending_state();
    }

    public void MDB_vend_approval(boolean approved){
        native_MDB_vend_approval(approved);
    }
    public Vending_State MDB_get_vending_state(){
        return Vending_State.by_code(native_MDB_get_vending_state());
    }

    public int MDB_get_item_price(){
        return native_MDB_get_item_price();
    }

    public void MDB_set_simulation_mode(Simulation_Mode simulation_mode){
        native_MDB_set_simulation_mode(simulation_mode.code);
    }

    private native void native_MDB_slave_func();
    private native int native_MDB_get_max_price();
    private native void native_MDB_set_user_funds(int user_funds) ;
    private native void native_MDB_start_vending_state();
    private native int native_MDB_get_vending_state();
    private native int native_MDB_get_item_price();
    private native void native_MDB_end_vend_state();
    private native void native_MDB_vend_approval(boolean approved);
    private native void native_MDB_set_simulation_mode(int simulation_mode);



};