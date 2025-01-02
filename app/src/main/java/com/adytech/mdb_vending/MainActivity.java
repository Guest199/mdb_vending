package com.adytech.mdb_vending;


import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import java.util.concurrent.TimeUnit;

public class MainActivity extends AppCompatActivity {

    MDB_Vending mdb_vending;
    String TAG = "yaniv";
    TextView textViewAlert;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.main_activity_layout);

Log.i(TAG, "Process start ");
        Button button = findViewById(R.id.begin_session);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                textViewAlert.setText("User tapped the BEGIN SESSION");
                mdb_vending.MDB_trigger_begin_session();
            }
        });

        textViewAlert = findViewById(R.id.alert);

        mdb_vending = new MDB_Vending();
        mdb_vending.MDB_start_driver();
        mdb_vending.MDB_set_simulation_mode(Simulation_Mode.NO_SIMULATION);//REGULAR_SEQUENCE_SCENARIO

Log.i(TAG, "MAX PRICE IS: "+ mdb_vending.MDB_get_max_price() );

        new  Thread(new Runnable() {
            @Override
            public void run() {
                doRun();
            }
        }).start();

    }

    private void doRun() {
        int max_price = mdb_vending.MDB_get_max_price();
//        int i = 1;
        while(true) {
            mdb_vending.MDB_set_user_funds(max_price);
            mdb_vending.MDB_start_vending_state();

            Vending_State vend_state_former = null, vend_state = null;

            while ( (vend_state = mdb_vending.MDB_get_vending_state()) == Vending_State.VEND_STATE_WAITING_SELECTION)
                if (vend_state_former != vend_state){
                    vend_state_former = vend_state;
                    setAlert("Vend state :  " + vend_state);
                }else
                    doWait(1);


            int amount = mdb_vending.MDB_get_item_price();
            setAlert("Vend state 2 " + vend_state + " amount is " + amount + " Waiting approval... ");
            //msg_dialog.setMessage("Waiting approval... price is " + amount);

            doWait(1);
//            mdb_vending.MDB_set_user_funds(max_price);
            mdb_vending.MDB_vend_approval(true);
            doWait(1);

            while (vend_state == Vending_State.VEND_STATE_SUPPLYING) {
                vend_state = mdb_vending.MDB_get_vending_state();
Log.i(TAG, "Vend state 3 " + vend_state);
                doWait(1);
            }
            mdb_vending.MDB_set_user_funds(0);
            mdb_vending.MDB_end_vend_state();
// Sleep 20 seconds
            doWait(20);

        }
    }

    void setAlert(String msg){
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                textViewAlert.setText(msg);
            }
        });

    }

    private void doWait(long timeout){
        try {
            TimeUnit.SECONDS.sleep(timeout); // Sleep one sec
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}