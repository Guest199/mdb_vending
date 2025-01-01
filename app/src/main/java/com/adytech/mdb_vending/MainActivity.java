package com.adytech.mdb_vending;


import android.app.Activity;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.Button;


import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.util.concurrent.TimeUnit;

public class MainActivity extends AppCompatActivity {

    MDB_Vending mdb_vending;
    Button button;


    File mdbport;
    private FileInputStream mSerR;
    private FileOutputStream mSerW;


    @Override
    protected void onCreate(Bundle savedInstanceState) {

        setTheme(android.support.constraint.R.style.Base_Theme_AppCompat);
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main_activity_layout);

        /*final AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
        final AlertDialog msg_dialog = builder.create();
        msg_dialog.show();
        msg_dialog.setTitle("MDB_VENDING");*/

        int i = 10;

        button = findViewById(R.id.begin_session);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                Log.d("BUTTONS", "User tapped the BEGIN SESSION");
                mdb_vending.MDB_trigger_begin_session();
            }
        });

        //msg_dialog.setCancelable(false);
        //msg_dialog.show();
        mdb_vending = new MDB_Vending();


        mdb_vending.MDB_start_driver();

        mdb_vending.MDB_set_simulation_mode(Simulation_Mode.NO_SIMULATION);//REGULAR_SEQUENCE_SCENARIO
        /*while (mdb_vending.MDB_get_max_price() == 0) {
            try {
                Thread.sleep(1000); // One sec
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }*/

        int max_price = mdb_vending.MDB_get_max_price();

        Log.i("*****MDB_VENDING", "MAX PRICE IS:"+ mdb_vending.MDB_get_max_price());


        //msg_dialog.setMessage("Max price is " + max_price);
        //msg_dialog.show();

        while(true) {
            mdb_vending.MDB_set_user_funds(max_price);
            mdb_vending.MDB_start_vending_state();

            Vending_State vend_state = mdb_vending.MDB_get_vending_state();
            while (vend_state == Vending_State.VEND_STATE_WAITING_SELECTION)  {
                    vend_state = mdb_vending.MDB_get_vending_state();
               // Log.i("MDB_VENDING", "Vend state is " + vend_state);

                i+=1;
               try {
                    TimeUnit.SECONDS.sleep(1); // Sleep one sec
                    Log.i("MDB_VENDING", "Vend state is %d:  " + vend_state +" " +  i);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                //msg_dialog.setMessage("Please select: ");

           }



            int amount = mdb_vending.MDB_get_item_price();
            Log.i("MDB_VENDING", "Vend state is " + vend_state + " amount is " + amount);
            Log.i("MDB_VENDING", "Waiting approval... ");
            //msg_dialog.setMessage("Waiting approval... price is " + amount);

            try {
                TimeUnit.SECONDS.sleep(1); // Sleep one sec
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            mdb_vending.MDB_set_user_funds(max_price);
            mdb_vending.MDB_vend_approval(true);
            //msg_dialog.setMessage("CONFIRMED! ");
            try {
                TimeUnit.SECONDS.sleep(1); // Sleep one sec
            } catch (InterruptedException e) {
                e.printStackTrace();
            }


            while (vend_state == Vending_State.VEND_STATE_SUPPLYING) {
                vend_state = mdb_vending.MDB_get_vending_state();
                Log.i("MDB_VENDING", "Vend state is " + vend_state);
                try {
                    TimeUnit.SECONDS.sleep(1); // Sleep one sec
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            mdb_vending.MDB_set_user_funds(0);
            mdb_vending.MDB_end_vend_state();
// Sleep 20 seconds
            try {
                TimeUnit.SECONDS.sleep(20); // Sleep 20 sec
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            int z = 99;

        }

//        super.onCreate(savedInstanceState);
    }
}