


FROM BUTTON TO CPP
---------------

MainActivity.java

        button = findViewById(R.id.begin_session);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                mdb_vending.MDB_trigger_begin_session();
        }});

MDB_Vending.java

    public void MDB_trigger_begin_session(){
        native_MDB_trigger_begin_session();
    }

    private native void native_MDB_trigger_begin_session();

cpp\native_interface.cpp

    extern "C"
    JNIEXPORT void JNICALL
    Java_com_adytech_mdb_1vending_MDB_1Vending_native_1MDB_1trigger_1begin_1session(JNIEnv *env,
                                                                                    jobject thiz) {
        MDB_Driver::get_instance().trigger_begin_session();
    }

cpp\MDB_Driver.h

    public:
        void trigger_begin_session();

cpp\MDB_Driver.cpp

    void MDB_Driver::trigger_begin_session()
    {
        MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
        m_csh_poll_state = CSH_BEGIN_SESSION_POLL_STATE;
    }


void MDB_Driver::mdb_handle_poll(void)
{
MDB_LOG(LOG_INFO, "POLL requested");
//ADDED
static int ack_cntr = 0, status =0;
// Read checksum
uint16_t vmc_chk;
bool ret = mdb_read(&vmc_chk);
switch (m_csh_poll_state) {
.
.
.

        case CSH_ACK_POLL_STATE:
            // if no data is to send, answer with ACK
            MDB_LOG(LOG_INFO, "m_csh_poll_state = CSH_ACK");
            mdb_write(CSH_ACK);
            ack_cntr++;
            if (ack_cntr == 100) {
                m_csh_poll_state = CSH_BEGIN_SESSION_POLL_STATE;
                MDB_LOG(LOG_INFO, "***********m_csh_poll_state %d => CSH_BEGIN_SESSION_POLL_STATE", ack_cntr);
            }

            break;


PROCESS OF LOOP ON ANDROID
------------

// START OBJECT	
	mdb_vending = new MDB_Vending();
	mdb_vending.MDB_start_driver();
	mdb_vending.MDB_set_simulation_mode(Simulation_Mode.NO_SIMULATION);//REGULAR_SEQUENCE_SCENARIO
	int max_price = mdb_vending.MDB_get_max_price();
	
// IN LOOP	
	mdb_vending.MDB_set_user_funds(max_price);
	mdb_vending.MDB_start_vending_state();
	
	mdb_vending.MDB_get_vending_state();
	mdb_vending.MDB_get_item_price();

	mdb_vending.MDB_set_user_funds(max_price);
	mdb_vending.MDB_vend_approval(true);

	mdb_vending.MDB_set_user_funds(0);
	mdb_vending.MDB_end_vend_state();

PROCESS OF LOOP ON CPP
------------

:mdb_slave_func()
	// OPEN mdb connection and starts loop

