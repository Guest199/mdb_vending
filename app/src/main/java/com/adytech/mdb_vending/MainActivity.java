package com.adytech.mdb_vending;


import android.app.Activity;
import android.app.AlertDialog;
import android.os.Bundle;
import android.util.Log;


import java.util.concurrent.TimeUnit;

public class MainActivity extends Activity {

    MDB_Vending mdb_vending;
    String TAG = "yaniv";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        final AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
        final AlertDialog msg_dialog = builder.create();
        msg_dialog.setTitle("MDB_VENDING");

        msg_dialog.setCancelable(false);
        msg_dialog.show();
        mdb_vending = new MDB_Vending();

        mdb_vending.MDB_start_driver();
        mdb_vending.MDB_set_simulation_mode(Simulation_Mode.NO_SIMULATION);
        while (mdb_vending.MDB_get_max_price() == 0) {
            dowait(1000);

//            try {
//                Thread.sleep( 1000); // One sec
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
        }

        int  max_price = mdb_vending.MDB_get_max_price();

        msg_dialog.setMessage("Max price is " + max_price);
        msg_dialog.show();

        System.out.println("Max price is " + max_price);


long l = 0l;
        while (true) {
            l = System.currentTimeMillis();
            mdb_vending.MDB_set_user_funds(max_price);
            mdb_vending.MDB_start_vending_state();

            Vending_State vend_state = mdb_vending.MDB_get_vending_state();
            Vending_State vend_state_former = vend_state;
            while (vend_state == Vending_State.VEND_STATE_WAITING_SELECTION) {
                vend_state = mdb_vending.MDB_get_vending_state();
Log.i(TAG, "Vend state is " + vend_state);
                dowait(1);
//                try {
//                    TimeUnit.SECONDS.sleep(1); // Sleep one sec
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
                msg_dialog.setMessage("Please select ");
            }

            int amount = mdb_vending.MDB_get_item_price();
Log.i(TAG, "Vend state is " + vend_state + " amount is " + amount);
Log.i(TAG, "Waiting approval... ");
            msg_dialog.setMessage("Waiting approval... price is " + amount);

            dowait(10);
//            try {
//                TimeUnit.SECONDS.sleep(10); // Sleep one sec
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }

            mdb_vending.MDB_set_user_funds(max_price);
            mdb_vending.MDB_vend_approval(true);
            msg_dialog.setMessage("CONFIRMED! ");
            dowait(1);
//            try {
//                TimeUnit.SECONDS.sleep(1); // Sleep one sec
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }


            while (vend_state == Vending_State.VEND_STATE_SUPPLYING) {
                vend_state = mdb_vending.MDB_get_vending_state();
Log.i(TAG, "Vend state is " + vend_state );

                dowait(1);
//                try {
//                    TimeUnit.SECONDS.sleep(1); // Sleep one sec
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
            }
Log.i(TAG, "Vend state is " + vend_state + " in " + (System.currentTimeMillis() - l));
            mdb_vending.MDB_set_user_funds(0);
            mdb_vending.MDB_end_vend_state();
// Sleep 20 seconds
            dowait(20);
//            try {
//                TimeUnit.SECONDS.sleep(20); // Sleep 20 sec
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//            int z = 99;
        }

//        super.onCreate(savedInstanceState);
    }

    private void dowait(long timeout){
        try {
            TimeUnit.SECONDS.sleep(timeout);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}