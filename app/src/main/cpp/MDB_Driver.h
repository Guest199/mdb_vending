//
// Created by zeevm on 19/02/2023.
//

#ifndef ABSEMV_ENGINE_MDB_DRIVER_H
#define ABSEMV_ENGINE_MDB_DRIVER_H

#include <string>
#include "Simulation_Mode.h"

class MDB_Driver {
    typedef enum VMC_Command{
        VMC_CMD_RESET =     0x0110,
        VMC_CMD_SETUP =     0x0111,
        VMC_CMD_POLL =      0x0112,
        VMC_CMD_VEND =      0x0113,
        VMC_CMD_READER =    0x0114,
        VMC_CMD_EXPANSION = 0x0117
    } VMC_Command;

    inline std::string to_string(VMC_Command cmd)
    {
        switch (cmd)
        {
            case VMC_CMD_RESET: return "VMC_CMD_RESET"; break;
            case VMC_CMD_SETUP: return "VMC_CMD_SETUP"; break;
            case VMC_CMD_POLL: return "VMC_CMD_POLL"; break;
            case VMC_CMD_VEND: return "VMC_CMD_VEND"; break;
            case VMC_CMD_READER: return "VMC_CMD_READER"; break;
            case VMC_CMD_EXPANSION: return "VMC_CMD_EXPANSION"; break;
            default:
                return "UNKNOWN";
        }
    }

    typedef enum Cashless_Reply{
        /*
 * MDB Level 01 Cashless device Replies
 * These are to be written in USART TX Buffer
 * Store them with MDB_Send
 * Don't change, written as in standard
 */
        CSH_ACK                     = 0x0100, // Acknowledgement, Mode-bit is set
        CSH_NAK                     = 0x01FF, // Negative Acknowledgement, Mode-bit is set
        CSH_SILENCE                 = 0xFFFF, // This one is not from standard, it's an impossible value for the VMC
        CSH_JUST_RESET              = 0x0000,
        CSH_RESTARTED               = 0x0010,
        CSH_READER_CONFIG_INFO      = 0x0001,
        CSH_DISPLAY_REQUEST         = 0x0002,
        CSH_BEGIN_SESSION           = 0x0003,
        CSH_SESSION_CANCEL_REQUEST  = 0x0004,
        CSH_VEND_APPROVED           = 0x0005,
        CSH_VEND_DENIED             = 0x0006,
        CSH_END_SESSION             = 0x0007,
        CSH_CANCELLED               = 0x0008,
        CSH_PERIPHERAL_ID           = 0x0009,
        CSH_MALFUNCTION_ERROR       = 0x000A,
        CSH_CMD_OUT_OF_SEQUENCE     = 0x000B,
        CSH_DIAGNOSTIC_RESPONSE     = 0x00FF,
    } Cashless_Reply;

/*
 * MDB VMC Subcommands
 */
    typedef enum VMC_Setup_Subcmd {
        VMC_SETUP_CONFIG_DATA    = 0x00,
        VMC_SETUP_MAX_MIN_PRICES = 0x01,
    } VMC_Setup_Subcmd;
    typedef enum VMC_Vend_Subcmd {
        VMC_VEND_REQUEST          = 0x00,
        VMC_VEND_CANCEL           = 0x01,
        VMC_VEND_SUCCESS          = 0x02,
        VMC_VEND_FAILURE          = 0x03,
        VMC_VEND_SESSION_COMPLETE = 0x04,
        VMC_VEND_CASH_SALE        = 0x05,
    } VMC_Vend_Subcmd;
    typedef enum VMC_Reader_Subcmd {
        VMC_READER_DISABLE = 0x00,
        VMC_READER_ENABLE  = 0x01,
        VMC_READER_CANCEL  = 0x02,
    } VMC_Reader_Subcmd;
    typedef enum VMC_Expansion_Subcmd {
        VMC_EXPANSION_REQUEST_ID  = 0x00,
        VMC_EXPANSION_DIAGNOSTICS = 0xFF,
    } VMC_Expansion_Subcmd;

    typedef enum Poll_State{
        CSH_ACK_POLL_STATE,
        CSH_SILENCE_POLL_STATE,
        CSH_JUST_RESET_POLL_STATE,
        CSH_RESTARTED_POLL_STATE,
        CSH_READER_CONFIG_INFO_POLL_STATE,
        CSH_DISPLAY_REQUEST_POLL_STATE,
        CSH_BEGIN_SESSION_POLL_STATE,
        CSH_SESSION_CANCEL_REQUEST_POLL_STATE,
        CSH_VEND_APPROVED_POLL_STATE,
        CSH_VEND_DENIED_POLL_STATE,
        CSH_END_SESSION_POLL_STATE,
        CSH_CANCELLED_POLL_STATE,
        CSH_PERIPHERAL_ID_POLL_STATE,
        CSH_MALFUNCTION_ERROR_POLL_STATE,
        CSH_CMD_OUT_OF_SEQUENCE_POLL_STATE,
        CSH_DIAGNOSTIC_RESPONSE_POLL_STATE,
    } Poll_State;

    inline std::string to_string_poll(Poll_State cmd)
    {
        switch (cmd)
        {
            case CSH_ACK_POLL_STATE: 					return "CSH_ACK_POLL_STATE"; break;
            case CSH_SILENCE_POLL_STATE: 				return "CSH_SILENCE_POLL_STATE"; break;
            case CSH_JUST_RESET_POLL_STATE: 			return "CSH_JUST_RESET_POLL_STATE"; break;
            case CSH_RESTARTED_POLL_STATE: 				return "CSH_RESTARTED_POLL_STATE"; break;
            case CSH_READER_CONFIG_INFO_POLL_STATE: 	return "CSH_READER_CONFIG_INFO_POLL_STATE"; break;
            case CSH_DISPLAY_REQUEST_POLL_STATE: 		return "CSH_DISPLAY_REQUEST_POLL_STATE"; break;
            case CSH_BEGIN_SESSION_POLL_STATE: 			return "CSH_BEGIN_SESSION_POLL_STATE"; break;
            case CSH_SESSION_CANCEL_REQUEST_POLL_STATE: return "CSH_SESSION_CANCEL_REQUEST_POLL_STATE"; break;
            case CSH_VEND_APPROVED_POLL_STATE: 			return "CSH_VEND_APPROVED_POLL_STATE"; break;
            case CSH_VEND_DENIED_POLL_STATE: 			return "CSH_VEND_DENIED_POLL_STATE"; break;
            case CSH_END_SESSION_POLL_STATE: 			return "CSH_END_SESSION_POLL_STATE"; break;
            case CSH_CANCELLED_POLL_STATE: 				return "CSH_CANCELLED_POLL_STATE"; break;
            case CSH_PERIPHERAL_ID_POLL_STATE: 			return "CSH_PERIPHERAL_ID_POLL_STATE"; break;
            case CSH_MALFUNCTION_ERROR_POLL_STATE: 		return "CSH_MALFUNCTION_ERROR_POLL_STATE"; break;
            case CSH_CMD_OUT_OF_SEQUENCE_POLL_STATE: 	return "CSH_CMD_OUT_OF_SEQUENCE_POLL_STATE"; break;
            case CSH_DIAGNOSTIC_RESPONSE_POLL_STATE: 	return "CSH_DIAGNOSTIC_RESPONSE_POLL_STATE"; break;
            default:
                return "UNKNOWN";
        }
    }

    typedef enum CSH_States {
        /* Level 01 Cashless (CSH) device
        states
        */
        CSH_STATE_INACTIVE     = 0x00,
        CSH_STATE_DISABLED     = 0x01,
        CSH_STATE_ENABLED      = 0x02,
        CSH_STATE_SESSION_IDLE = 0x03,
        CSH_STATE_VEND         = 0x04,
        // this one is to avoid multiple calls of functions that were already executed once
        CSH_STATE_PROCESSING   = 0xFF,
    } CSH_States;

    class VMC_Config
    {
    public:

        uint8_t featureLevel;
        uint8_t displayColumns;
        uint8_t displayRows;
        uint8_t displayInfo;

        VMC_Config(uint8_t featureLevel, uint8_t displayColumns, uint8_t displayRows, uint8_t displayInfo)
        {
            this->featureLevel = featureLevel;
            this->displayColumns = displayColumns;
            this->displayRows = displayRows;
            this->displayInfo = displayInfo;
        }
    };

    class CSH_Config
    {
    public:
        uint8_t featureLevel;
        uint8_t countryCodeH;
        uint8_t countryCodeL;
        uint8_t scaleFactor;
        uint8_t decimalPlaces;
        uint8_t maxResponseTime; // seconds, overrides default NON-RESPONSE time
        uint8_t miscOptions;

        CSH_Config(
                uint8_t featureLevel,
                uint8_t countryCodeH,
                uint8_t countryCodeL,
                uint8_t scaleFactor,
                uint8_t decimalPlaces,
                uint8_t maxResponseTime, // seconds, overrides default NON-RESPONSE time
                uint8_t miscOptions)
        {
            this->featureLevel = featureLevel;
            this->countryCodeH = countryCodeH;
            this->countryCodeL = countryCodeL;
            this->scaleFactor = scaleFactor;
            this->decimalPlaces = decimalPlaces;
            this->maxResponseTime = maxResponseTime; // seconds, overrides default NON-RESPONSE time
            this->miscOptions = miscOptions;
        }
    };

    class VMC_Prices
    {
    public:
        uint16_t maxPrice;
        uint16_t minPrice;

        VMC_Prices(uint16_t maxPrice, uint16_t minPrice)
        {
            this->maxPrice = maxPrice;
            this->minPrice = minPrice;
        }
    };
    
public:
    typedef enum MDB_Mode {
            MDB_MASTER,
            MDB_PERIPHERAL
    } MDB_Mode;

    typedef enum Vend_State{
        VEND_STATE_IDLE             = 0,
        VEND_STATE_WAITING_SELECTION= 1,
        VEND_STATE_WAITING_APPROVAL = 2,
        VEND_STATE_SUPPLYING        = 3,
        VEND_STATE_FAILED           = 4,
        VEND_STATE_DENIED           = 5,
        VEND_STATE_SUCCEEDED        = 6,
    };

    static MDB_Driver &get_instance();

    void mdb_slave_func();

    Vend_State MDB_get_vend_state(){ return m_vend_state;};

private:
    MDB_Driver();

    int m_fd;
    Simulation_Mode m_simulation_mode = NO_SIMULATION;
    Vend_State m_vend_state = VEND_STATE_IDLE;
    VMC_Config m_vmc_config = VMC_Config(0, 0, 0, 0);
    VMC_Prices m_vmc_prices = VMC_Prices(0, 0);
    Poll_State m_csh_poll_state = CSH_RESTARTED_POLL_STATE;
    Poll_State m_csh_poll_state_former;
    uint8_t m_csh_state = CSH_STATE_INACTIVE;
    // Available user funds, changed via server connection in the Enabled state
    uint16_t m_user_funds  = 0x0000;
    // Selected item price,  changed via VMC_VEND_REQUEST
    uint16_t m_item_price   = 0x0000;
    uint16_t m_item_number = 0x0000;
    // Selected item amount, changed via VMC_VEND_REQUEST
    // This one is not necessarily the amount of selected items to dispense,
    // it also can be a single item position value in the VMC memory
    uint16_t m_vend_amount = 0x0000;
    uint8_t m_csh_error_code = 0;
    bool    m_debug_on = false;
    CSH_Config m_csh_config = {
            0x01, // featureLevel
            0x03, // countryCode
            0x76,
            0x00, // Scale Factor = 100
            0x02, // Decimal Places
            90, // Max Response Time, seconds
            0b00001101  // Misc Options
/*b0=0: The payment media reader is NOT capable of restoring funds
to the user’s payment media or account. Do not request
refunds.
b0=1: The payment media reader is capable of restoring funds to the
user’s payment media or account. Refunds may be requested.
b1=0: The payment media reader is NOT multivend capable.
Terminate session after each vend.
b1=1: The payment media reader is multivend capable. Multiple
items may be purchased within a single session.
b2=0: The payment media reader does NOT have a display.
b2=1: The payment media reader does have its own display.
b3=0: The payment media reader does NOT support the
VEND/CASH SALE subcommand.
b3=1: The payment media reader does support the VEND/CASH
SALE subcommand.
b4-b7=0 Any future options must be covered by the EXPANSION
COMMAND option bits.*/
    };

    bool mdb_read(uint16_t* data);
    bool mdb_write(const uint16_t data);
    bool mdb_read(uint16_t* data, char* msg);
    bool mdb_write(const uint16_t data, char* msg);
    void mdb_handle_setup();
    uint8_t calc_checksum(uint8_t *array, uint8_t arr_size);
    void send_config_info();
    int mdb_open(const std::string& dev_name);
    int mdb_set_mode(int fd, int mode);
    int mdb_flush(int fd, int queue_selector);
    int mdb_close(int fd);
    int mdbp_set_addr(int fd, unsigned char *addr, unsigned int size);
    int mdbp_set_interval(int fd, unsigned int interval);
    void set_priority();
    void mdb_handle_reset();
    void send_peripheral_ID(void);
    void mdb_handle_poll(void);
    void mdb_handle_vend();
    void vend_request(void);
    void mdb_handle_reader();
    void mdb_handle_expansion();
    void disable(void);
    void enable(void);
    void expansion_request_ID();
    void expansion_diagnostics();
    void vend_denied();
    void vend_success_handler(void);
    void vend_failure_handler(void);
    void vend_session_complete(void);
    void vend_cash_sale(void);
    void cancelled(void);
    int set_process_priority(pid_t pid);


public:
    void set_user_funds(uint16_t user_funds);
    void start_vending_state();
    bool is_vend_succeeded();
    uint16_t get_max_price();
    uint16_t get_item_price();
    uint16_t get_amount_of_items();
    void end_vending_state();
    void trigger_begin_session();

    void vend_approval(bool approved);
    void set_simulation_mode(int simulation_mode){ m_simulation_mode = (Simulation_Mode)simulation_mode; };
};


#endif //ABSEMV_ENGINE_MDB_DRIVER_H
