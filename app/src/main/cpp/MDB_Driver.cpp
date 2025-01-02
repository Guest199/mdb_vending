//
// Created by zeevm on 19/02/2023.
//

#include "MDB_Driver.h"

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <thread>
#include <sys/syscall.h>
#include <android/log.h>
#include <syslog.h>

#define LOG_MODE true


#if (LOG_MODE == true)
#define MDB_LOG syslog
#else
#define MDB_LOG //
#endif
#define MODE_SET_BIT (1<<8)

#define MDB_BUF_I (1<<0)
#define MDB_BUF_O (1<<1)
#define MDB_BUF_IO (MDB_BUF_I|MDB_BUF_O)

#define MDB_MAGIC 'm'
#define IOCTL_MDB_SET_MODE			_IOW(MDB_MAGIC, 1, int)
#define IOCTL_MDB_FLUSH_BUF 		_IOW(MDB_MAGIC, 2, int)
#define IOCTL_MDBP_SET_ADDR			_IOW(MDB_MAGIC, 3, unsigned char)
#define IOCTL_MDB_SET_INTERVAL		_IOW(MDB_MAGIC, 4, int)

#define MAX_MDB_ADDR_SIZE 16
#define MDB_DEV_NAME "/dev/mdb_slave"
#define MDB_LOGGER "MDB_LOG"
#define ERR_P(fmt,args...) do {char L_sbuf[1024]={0};snprintf(L_sbuf,sizeof(L_sbuf),"%s%s%s","[%s][%d]:",fmt,"\n");printf(L_sbuf,__FUNCTION__,__LINE__,##args);fflush(stdout);} while (0)
#define ERR(fmt,args...) do {ERR_P(fmt,##args);usleep(200);} while (0)

//extern "C" {
//int pax_SetThreadPriority(int callingPid);
//}

struct mdb_addr{
    int addr_size;
    unsigned char addr_data[16];
};

MDB_Driver::MDB_Driver()
{
//    std::thread mdb_slave_thread(&MDB_Driver::mdb_slave_func, this);

//    uint16_t max_price = get_max_price();
//    mdb_slave_thread.join();
//    sched_param sch_params;
//    sch_params.sched_priority = priority;
//    if(pthread_setschedparam(th.native_handle(), policy, &sch_params)) {
//        std::cerr << "Failed to set Thread scheduling : " << std::strerror(errno) << std::endl;
//    }
}

MDB_Driver& MDB_Driver::get_instance()
{
    static MDB_Driver instance;
    return instance;
}

int MDB_Driver::mdb_open(const std::string& dev_name)
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    errno = 0;

    return open(dev_name.c_str(), O_RDWR, 0);
}

int MDB_Driver::mdb_set_mode(int fd, int mode)
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    errno = 0;
    return ioctl(fd, IOCTL_MDB_SET_MODE, mode);
}



bool MDB_Driver::mdb_write(const uint16_t data, char* msg){
    __android_log_print(ANDROID_LOG_INFO, "MDB_READ_WRITE","10. mdb write 0x%04X %s" , data, msg);
    return MDB_Driver::mdb_write(data);
}

bool MDB_Driver::mdb_read(uint16_t* data, char* msg) {
    uint16_t initial_value = *data;

    bool result = read(m_fd, (char*)data, 2) == 2;

    if (initial_value != data_former || *data != data_rs_former) {
        __android_log_print(ANDROID_LOG_INFO, "MDB_READ_WRITE",
                            "11. read %s call: 0x%04X, result: 0x%04X", msg, initial_value, *data);
        data_former = initial_value;
        data_rs_former = *data;
    }
    return result;
}

//bool MDB_Driver::mdb_read(uint16_t* data, char* msg){
//    __android_log_print(ANDROID_LOG_INFO, "MDB_READ_WRITE", "11. mdb read 0x%04X %s" , data, msg);
//    return MDB_Driver::mdb_read(data);
//}

bool MDB_Driver::mdb_read(uint16_t* data){
    return read(m_fd, (char*)data, 2) == 2;
}

bool MDB_Driver::mdb_write(const uint16_t data)
{
    errno = 0;
//        if (m_debug_on) {
//            MDB_LOG(LOG_INFO, "mdb_write 0x%04X", data);
//        }
//        __android_log_print(ANDROID_LOG_INFO, "MDB_READ_WRITE","10. mdb write 0x%04X", data);
    return write(m_fd, (char*)&data, 2) == 2;
}

int MDB_Driver::mdb_flush(int fd, int queue_selector)
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    int sel = 0;
    errno = 0;

    if(queue_selector < 0){
        errno = EINVAL;
        return -1;
    }

    if(queue_selector == MDB_BUF_O
       || queue_selector == MDB_BUF_I
       || queue_selector == (MDB_BUF_O | MDB_BUF_I)){
        if(queue_selector & MDB_BUF_O){
            sel |= MDB_BUF_O;
        }

        if(queue_selector & MDB_BUF_I){
            sel |= MDB_BUF_I;
        }
    }else{
        errno = EINVAL;
        return -1;
    }

    return ioctl(fd, IOCTL_MDB_FLUSH_BUF, sel);
}

int MDB_Driver::mdb_close(int fd)
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    errno = 0;
    return close(fd);
}

int MDB_Driver::mdbp_set_addr(int fd, unsigned char *addr, unsigned int size)
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    struct mdb_addr mdbp_addr;
    int i = 0;

    errno = 0;
    if(size > MAX_MDB_ADDR_SIZE){
        errno = -EINVAL;
        return -1;
    }

    memset((void*)&mdbp_addr, 0, sizeof(mdbp_addr));
    mdbp_addr.addr_size = size;

    for(i=0; i<size; i++){
        mdbp_addr.addr_data[i] = addr[i];
    }

    return ioctl(fd, IOCTL_MDBP_SET_ADDR, &mdbp_addr);
}

int MDB_Driver::mdbp_set_interval(int fd, unsigned int interval)
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    return ioctl(fd, IOCTL_MDB_SET_INTERVAL, &interval);
}

/*
 * Handles Setup sequence
 */
void MDB_Driver::mdb_handle_setup()
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    char buff[2];
//    for (int i=0; i<6; i++)
//    {
//        int ret = read(m_fd, buff, 2);
//        MDB_LOG(LOG_INFO, "mdb_handle_setup %d read: 0x%02x 0x%02x", ret, buff[0], buff[1]);
//    }
//    return;

    uint8_t i; // counter
    uint8_t checksum = (uint8_t)(VMC_CMD_SETUP & 0x00FF);
    uint16_t vmc_temp;
    uint8_t vmc_data[6];
    /*
     * wait for the whole frame
     * frame structure:
     * 1 subcommand + 4 vmc_config fields + 1 Checksum byte
     * 6 elements total
     */
    int count = 0;
    while (count < 6)
    {
        uint16_t buff;
        if(mdb_read(&buff, "105"))
        {
            vmc_data[count++] = (uint8_t)(buff & 0x00ff);
        }
    }

    // calculate checksum excluding last read element, which is a received checksum
    // for (i = 0; i < 5; ++i)
    checksum += calc_checksum(vmc_data, 5);
    // compare calculated and received checksums
    if (checksum != vmc_data[5])
    {
        mdb_write(CSH_NAK, "28");
        MDB_LOG(LOG_ERR, "checksum err, calculated = %d, actual = %d", checksum, vmc_data[5]);
        return; // checksum mismatch, error
    }

    // vmc_data[0] is a SETUP Config Data or Max/Min Prices identifier
    switch(vmc_data[0])
    {
        case VMC_SETUP_CONFIG_DATA :
        {
            // Store VMC data
            m_vmc_config = {vmc_data[1], vmc_data[2], vmc_data[3], vmc_data[4]};
            MDB_LOG(LOG_INFO, "vmc_config.featureLevel %d, vmc_config.displayColumns %d, vmc_config.displayRows %d, vmc_config.displayInfo %d", m_vmc_config.featureLevel, m_vmc_config.displayColumns, m_vmc_config.displayRows, m_vmc_config.displayInfo);
            send_config_info();
        }; break;

        case VMC_SETUP_MAX_MIN_PRICES : {
            // Store VMC Prices
            uint16_t maxPrice = ((uint16_t)vmc_data[1] << 8) | vmc_data[2];
            uint16_t minPrice = ((uint16_t)vmc_data[3] << 8) | vmc_data[4];
            m_vmc_prices = {maxPrice, minPrice};
            // Send ACK
            mdb_write(CSH_ACK, "29");
            // Change state to DISABLED
            m_csh_state = CSH_STATE_DISABLED;
        }; break;

        default : break;
    }
}

void MDB_Driver::send_config_info(void)
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    uint8_t checksum = 0;
    // calculate checksum, no Mode bit yet
    checksum = ( (uint8_t)(CSH_READER_CONFIG_INFO & 0x00FF)
                 + m_csh_config.featureLevel
                 + m_csh_config.countryCodeH
                 + m_csh_config.countryCodeL
                 + m_csh_config.scaleFactor
                 + m_csh_config.decimalPlaces
                 + m_csh_config.maxResponseTime
                 + m_csh_config.miscOptions );

//    mdb_write(CSH_ACK);
    mdb_write(CSH_READER_CONFIG_INFO, "31 CONFIG");
    mdb_write(m_csh_config.featureLevel);
    mdb_write(m_csh_config.countryCodeH);
    mdb_write(m_csh_config.countryCodeL);
    mdb_write(m_csh_config.scaleFactor);
    mdb_write(m_csh_config.decimalPlaces);
    mdb_write(m_csh_config.maxResponseTime);
    mdb_write(m_csh_config.miscOptions);
    mdb_write(checksum | CSH_ACK);
}

void MDB_Driver::mdb_slave_func()
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );

// OPEN CONNECTION
    set_priority();
    const char *serialPort = MDB_DEV_NAME;
    const char *mdbLog = MDB_LOGGER;
    m_fd = open("/dev/mdb_slave", O_RDWR | O_NOCTTY, 0);

    if (m_fd == -1) {
        __android_log_print(ANDROID_LOG_INFO, mdbLog, "1. ++_**************__open_port: Unable to open MDB Port: %d -- %s ", m_fd, serialPort);
    }
    __android_log_print(ANDROID_LOG_INFO, mdbLog, "2. open fd = %d", m_fd);

//SET MODE
    int mode = MDB_PERIPHERAL;
    int ret_set_mode = ioctl(m_fd, IOCTL_MDB_SET_MODE, mode);
    __android_log_print(ANDROID_LOG_INFO, mdbLog, "3. IOCTL_MDB_SET_MODE = %d", ret_set_mode);

//SET ADDRESS
    struct mdb_addr {
        int addr_size;
        unsigned char addr_data[16];
    } arr;
    memset(&arr,0,sizeof(arr));
    arr.addr_size = 1;
    arr.addr_data[0]=0x10;
    int ret_set_addr = ioctl(m_fd,IOCTL_MDBP_SET_ADDR, &arr);
    __android_log_print(ANDROID_LOG_INFO, mdbLog,  "4. ret_set_addr = %d", ret_set_addr);

//FLUSH BUFFER
    int flush_sel = MDB_BUF_IO;
    int ret_flush = ioctl(m_fd, IOCTL_MDB_FLUSH_BUF, flush_sel);
    __android_log_print(ANDROID_LOG_INFO, mdbLog,  "5. ret_flush = %d", ret_flush);
    __android_log_print(ANDROID_LOG_INFO, mdbLog,  "6. mdb_slave_func start listening");
    char buff[120];
    char recv_buf[36]={0};
    bool b;


    uint16_t vmc_cmd , vmc_cmd_former;
    while (m_simulation_mode == NO_SIMULATION){
        bool ret = mdb_read(&vmc_cmd);
//        bool ret = mdb_read(&vmc_cmd, "107. mdb_slave_func");

        if (ret) {
            if (m_debug_on) {
                sprintf(buff, "ret %d read: 0x%04X", ret, vmc_cmd);
                MDB_LOG(LOG_INFO, "%s", buff);
            }

            if ((vmc_cmd >= VMC_CMD_RESET) && (vmc_cmd <= VMC_CMD_EXPANSION)) {

                if (vmc_cmd != vmc_cmd_former ) {
                    __android_log_print(ANDROID_LOG_INFO, mdbLog, "7. cmd = %s (0x%04X) 0x%04X",
                                        to_string((VMC_Command) vmc_cmd).c_str(), vmc_cmd , m_csh_poll_state);
                    vmc_cmd_former = vmc_cmd;
                }

                switch ((VMC_Command) vmc_cmd) {
                    case VMC_CMD_RESET:
                        mdb_handle_reset();
                        break;
                    case VMC_CMD_SETUP:
                        mdb_handle_setup();
                        break;
                    case VMC_CMD_POLL:
                        mdb_handle_poll();
                        break;
                    case VMC_CMD_VEND:
                        mdb_handle_vend();
                        break;
                    case VMC_CMD_READER:
                        mdb_handle_reader();
                        break;
                    case VMC_CMD_EXPANSION :
                        mdb_handle_expansion();
                        break;
                    default:
                        break;
                }
            }
        }
        else
        {
            sleep(1);
        }
    }


    close(m_fd);
}

int MDB_Driver::set_process_priority(pid_t pid)
{
    int maxpri = 0;
    int ret = 0;
    struct sched_param param;
    maxpri = sched_get_priority_max(SCHED_FIFO);
    if(maxpri == -1){
        return -1;
    }
    param.sched_priority = maxpri;
    ret = sched_setscheduler(pid, SCHED_FIFO, &param);
    if(ret == -1){
        return -1;
    }
    return 0;
}

void MDB_Driver::set_priority()
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    pid_t tid = syscall(SYS_gettid);
    MDB_LOG(LOG_INFO, "pid=%d", getpid());
    MDB_LOG(LOG_INFO, "tid=%d", tid);
//    int ret = pax_SetThreadPriority(tid);
//    ERR("pax_SetThreadPriority(%d)=%d",tid,ret);
    int ret = set_process_priority(tid);
    ERR("set_process_priority(%d)=%d", tid , ret);
}

/*
  * calc_checksum()
  * Calculates checksum of *array from 0 to arr_size
  * Use with caution (because of pointer arithmetics)
  */
uint8_t MDB_Driver::calc_checksum(uint8_t *array, uint8_t arr_size)
{
    uint8_t ret_val = 0x00;
    uint8_t i;
    for (i = 0; i < arr_size; ++i)
        ret_val += *(array + i);
    return ret_val;
}

void MDB_Driver::mdb_handle_reset()
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    // Read CHK byte
    uint16_t data;
    bool res = mdb_read(&data, "104");
//    uint8_t expected_chksum = VMC_CMD_RESET;
//    if(res && ((uint8_t)(data & 0x00ff)) == expected_chksum)
//    {
    MDB_LOG(LOG_INFO, "RESET requested");
    // Reset all data
    m_vmc_config.featureLevel   = 0;
    m_vmc_config.displayColumns = 0;
    m_vmc_config.displayRows    = 0;
    m_vmc_config.displayInfo    = 0;

    m_vmc_prices.maxPrice = 0;
    m_vmc_prices.minPrice = 0;

    m_csh_error_code = 0;
    // Send ACK, turn INACTIVE
    m_csh_state = CSH_STATE_INACTIVE;
    m_csh_poll_state = CSH_JUST_RESET_POLL_STATE;

    // Send Just_Reset back
    mdb_write(CSH_JUST_RESET, "26");
    mdb_write(CSH_ACK, "27");
//    }
}

/*
 * Handles Poll replies
 */
void MDB_Driver::mdb_handle_poll(void)
{
//ADDED
    static int ack_cntr = 0, status =0;

    // Read checksum
    uint16_t vmc_chk;
//    bool ret = mdb_read(&vmc_chk );
    bool ret = mdb_read(&vmc_chk , "103. mdb_handle_poll");

    if (m_csh_poll_state_former != m_csh_poll_state) {
        __android_log_print(ANDROID_LOG_INFO, "MDB_LOG",
                            "11. read = 0x%04X ,poll-state= %s (0x%04X)",
                            vmc_chk, to_string_poll((Poll_State) m_csh_poll_state).c_str(),
                            m_csh_poll_state);
        m_csh_poll_state_former = m_csh_poll_state;
    }

    switch (m_csh_poll_state) {
        case CSH_SILENCE_POLL_STATE:
            break;
        case CSH_READER_CONFIG_INFO_POLL_STATE:
            MDB_LOG(LOG_INFO, "m_csh_poll_state = CSH_READER_CONFIG_INFO");
            send_config_info();
            break;
        case CSH_DISPLAY_REQUEST_POLL_STATE:
            MDB_LOG(LOG_INFO, "m_csh_poll_state = CSH_DISPLAY_REQUEST");
            // Not implemented
            break; // <<<=== Global Display Message
        case CSH_RESTARTED_POLL_STATE:
            MDB_LOG(LOG_INFO, "m_csh_poll_state = CSH_RESTARTED");
            mdb_write(CSH_RESTARTED, "6");
            m_csh_poll_state = CSH_JUST_RESET_POLL_STATE;
            break;
        case CSH_ACK_POLL_STATE:
            // if no data is to send, answer with ACK
//            MDB_LOG(LOG_INFO, "m_csh_poll_state = CSH_ACK %d", ack_cntr);
            mdb_write(CSH_ACK );
//            mdb_write(CSH_ACK , "CSH_ACK_POLL_STATE");
            ack_cntr++;
            if (ack_cntr == 100) {
                m_csh_poll_state = CSH_BEGIN_SESSION_POLL_STATE;
                MDB_LOG(LOG_INFO, "***********m_csh_poll_state %d => CSH_BEGIN_SESSION_POLL_STATE", ack_cntr);
            }

            break;
        case CSH_JUST_RESET_POLL_STATE:
            MDB_LOG(LOG_INFO, "m_csh_poll_state = CSH_JUST_RESET");
            mdb_write(CSH_JUST_RESET, "7. CSH_JUST_RESET_POLL_STATE");
            send_config_info();
            m_csh_poll_state = CSH_ACK_POLL_STATE;
            break;
        case CSH_BEGIN_SESSION_POLL_STATE:
        {
            MDB_LOG(LOG_INFO, "m_csh_poll_state = CSH_BEGIN_SESSION");
            uint8_t checksum = 0;
            // Zero data of the former session
            m_item_price = 0;
            m_vend_amount = 0;

//            uint8_t user_funds_H = (uint8_t)(m_user_funds >> 8);
//            uint8_t user_funds_L = (uint8_t)(m_user_funds &  0x00FF);
            // Do not display available funds
            uint8_t user_funds_H = 0xFF;
            uint8_t user_funds_L = 0xFF;
            // calculate checksum, no Mode bit yet
            checksum = ( (uint8_t)(CSH_BEGIN_SESSION & 0x00FF)
                         + user_funds_H
                         + user_funds_L );

            mdb_write(CSH_BEGIN_SESSION, "8. CSH_BEGIN_SESSION");
            mdb_write(user_funds_H);    // upper byte of funds available
            mdb_write(user_funds_L);      // lower byte of funds available
            mdb_write(checksum | CSH_ACK); // set Mode bit and send
            m_csh_state = CSH_STATE_SESSION_IDLE;
        }
            break; // <<<=== Global User Funds
        case CSH_SESSION_CANCEL_REQUEST_POLL_STATE:
            MDB_LOG(LOG_INFO, "m_csh_poll_state = CSH_SESSION_CANCEL_REQUEST");
            mdb_write(CSH_SESSION_CANCEL_REQUEST, "12");
            mdb_write(CSH_SESSION_CANCEL_REQUEST | CSH_ACK, "13"); // Checksum with Mode bit
            break;
        case CSH_VEND_APPROVED_POLL_STATE:
        {
            MDB_LOG(LOG_INFO, "m_csh_poll_state = CSH_VEND_APPROVED");
            uint8_t checksum = 0;
            uint8_t vend_amount_H = (uint8_t) (m_vend_amount >> 8);
            uint8_t vend_amount_L = (uint8_t) (m_vend_amount & 0x00FF);
            // calculate checksum, no Mode bit yet
            checksum = ((uint8_t)(CSH_VEND_APPROVED & 0x00FF)
                        + vend_amount_H
                        + vend_amount_L);
            mdb_write(CSH_VEND_APPROVED, "14. CSH_VEND_APPROVED");
            mdb_write(vend_amount_H);       // Vend Amount H
            mdb_write(vend_amount_L);       // Vend Amount L
            mdb_write(checksum | CSH_ACK);  // set Mode bit and send
//            m_csh_poll_state = CSH_END_SESSION_POLL_STATE;
        }
            break;
        case CSH_VEND_DENIED_POLL_STATE:
            MDB_LOG(LOG_INFO, "m_csh_poll_state = CSH_VEND_DENIED");
            vend_denied();
            break;
        case CSH_END_SESSION_POLL_STATE:
            MDB_LOG(LOG_INFO, "m_csh_poll_state = CSH_END_SESSION");
            mdb_write(CSH_END_SESSION, "18. CSH_END_SESSION_POLL_STATE");
            MDB_LOG(LOG_INFO, "mdb_write 0x%04X", CSH_END_SESSION );
            mdb_write(CSH_END_SESSION | CSH_ACK, "19. CSH_END_SESSION_POLL_STATE"); // Checksum with Mode bit
            MDB_LOG(LOG_INFO, "mdb_write 0x%04X", CSH_END_SESSION | CSH_ACK);

            // Zero data of the former session
            m_user_funds = 0;

            m_csh_poll_state = CSH_JUST_RESET_POLL_STATE;
            m_csh_state = CSH_STATE_ENABLED;
            break;
        case CSH_CANCELLED_POLL_STATE:
            MDB_LOG(LOG_INFO, "m_csh_poll_state = CSH_CANCELLED");
            if (m_csh_state == CSH_STATE_ENABLED)
            {
                mdb_write(CSH_CANCELLED, "20");
                mdb_write(CSH_CANCELLED | CSH_ACK, "21"); // Checksum with Mode bit
            }
            break;
        case CSH_PERIPHERAL_ID_POLL_STATE:
            send_peripheral_ID();
            break;
        case CSH_MALFUNCTION_ERROR_POLL_STATE:
        {
            MDB_LOG(LOG_INFO, "m_csh_poll_state = CSH_MALFUNCTION_ERROR");
            uint8_t malf_err_msg[3];
            malf_err_msg[0] = CSH_MALFUNCTION_ERROR;
            malf_err_msg[1] = m_csh_error_code;
            // calculate checksum, set Mode bit and store it
            malf_err_msg[2] = calc_checksum(malf_err_msg, 2) | CSH_ACK;
            for (int i = 0; i < 3; ++i) {
                mdb_write(malf_err_msg[i], "22");
            }
        }
            break;
        case CSH_CMD_OUT_OF_SEQUENCE_POLL_STATE:
            MDB_LOG(LOG_INFO, "m_csh_poll_state = CSH_CMD_OUT_OF_SEQUENCE");
            mdb_write(CSH_CMD_OUT_OF_SEQUENCE, "23");
            mdb_write(CSH_CMD_OUT_OF_SEQUENCE | CSH_ACK, "24");
            break;
        case CSH_DIAGNOSTIC_RESPONSE_POLL_STATE:
            MDB_LOG(LOG_INFO, "m_csh_poll_state = CSH_DIAGNOSTIC_RESPONSE");
            // Not implemented yet
            break;
        default :
            break;
    }
}

void MDB_Driver::send_peripheral_ID(void)
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    // RETURNS DEFAULT VALUES, NOT SO UPDATED...
    // Manufacturer Data, 30 bytes + checksum with mode bit
    uint8_t i; // counter
    uint8_t checksum = 0;
    uint8_t periph_id[31];
    uint8_t a_char = 0x41;
    periph_id[0] = CSH_PERIPHERAL_ID;
    // Set Manufacturer ID ADY000000001
    periph_id[1] = 'A'; periph_id[2] = 'D'; periph_id[3] = 'Y'; // Adytek

    for (i = 4; i < 15; ++i)
        periph_id[i] = 0;

    periph_id[15] = 1;

    // Set Serial Number, ASCII
    // Set Model Number, ASCII
    for (i = 16; i < 28; ++i)
        periph_id[i] = a_char + i;
    // Set Software verion, packed BCD
    periph_id[28] = 1;
    periph_id[29] = 0;
    periph_id[30] = calc_checksum(periph_id, 29);
    // Send all data on the bus
    for (i = 0; i < 30; ++i)
        mdb_write(periph_id[i], "32");

    mdb_write(periph_id[30] | CSH_ACK, "33");
}

void MDB_Driver::mdb_handle_vend()
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    uint16_t subcomm_temp;
    uint8_t subcomm;

    // Store Subcommand in array
    mdb_read(&subcomm_temp, "106. mdb_handle_vend");
    subcomm = (uint8_t)(subcomm_temp & 0x00FF); // get rid of Mode bit if present
    // Switch through subcommands
    switch(subcomm)
    {
        case VMC_VEND_REQUEST : vend_request();         break;
        case VMC_VEND_CANCEL  : vend_denied();          break;
        case VMC_VEND_SUCCESS : vend_success_handler();  break;
        case VMC_VEND_FAILURE : vend_failure_handler();  break;
        case VMC_VEND_SESSION_COMPLETE : vend_session_complete(); break;
        case VMC_VEND_CASH_SALE : vend_cash_sale();      break;
        default : break;
    }
}

/*
 * Internal functions for MDB_VendHandler()
 */
void MDB_Driver::vend_request(void)
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    uint8_t i; // counter
    uint8_t checksum = (uint8_t)(VMC_CMD_VEND & 0x00FF) + VMC_VEND_REQUEST;
    uint8_t vend_data[5];
    uint16_t vend_temp;
    // Wait for 5 elements in buffer
    // 4 data + 1 checksum
    // Read all data and store it in an array, with a subcommand
    for (i = 0; i < 5; ++i)
    {
        mdb_read(&vend_temp, "110. vend_request");
        vend_data[i] = (uint8_t)(vend_temp & 0x00FF); // get rid of Mode bit if present
    }

    // calculate checksum excluding last read element, which is a received checksum
    checksum += calc_checksum(vend_data, 4);

    // compare calculated and received checksums
    if (checksum != vend_data[4])
    {
        mdb_write(CSH_NAK, "39. vend_request");
        return; // checksum mismatch, error
    }

    MDB_LOG(LOG_INFO, "vend_data = %02x %02x %02x %02x", vend_data[0], vend_data[1], vend_data[2], vend_data[3]);
    m_item_price = (vend_data[0] << 8) | vend_data[1];
    m_item_number = (vend_data[2] << 8) | vend_data[3];
    m_vend_amount = m_item_price;

    MDB_LOG(LOG_INFO, "item_cost = %d vend_amount = %d", m_item_price, m_vend_amount);

    // Send ACK to VMC
    mdb_write(CSH_ACK, "40. ");

    // Set uninterruptable VEND state
    m_csh_state = CSH_STATE_VEND;
    m_csh_poll_state = CSH_ACK_POLL_STATE;
    m_vend_state = VEND_STATE_WAITING_APPROVAL;
}

void MDB_Driver::vend_approval(bool approved)
{
    MDB_LOG(LOG_INFO, "%s approved %d", __PRETTY_FUNCTION__, approved );
    m_csh_poll_state = approved ? CSH_VEND_APPROVED_POLL_STATE : CSH_VEND_DENIED_POLL_STATE;
    m_vend_state = VEND_STATE_SUPPLYING;
    if (m_simulation_mode != NO_SIMULATION) {
        std::thread supplying_thread = std::thread([](MDB_Driver *driver) {
            MDB_LOG(LOG_INFO, "sleep_for 10 sec to simulate supplying");
            std::this_thread::sleep_for(std::chrono::seconds(10));
            if (driver->m_simulation_mode == FAILURE_IN_SUPPLYING_SCENARIO)
            {
                MDB_LOG(LOG_INFO, "setting vend state to VEND_STATE_SUCCEEDED");
                driver->m_vend_state = VEND_STATE_FAILED;
            }
            else
            {
                MDB_LOG(LOG_INFO, "setting vend state to VEND_STATE_SUCCEEDED");
                driver->m_vend_state = VEND_STATE_SUCCEEDED;
            }
        }, this);
        supplying_thread.detach();
    }
}

void MDB_Driver::vend_failure_handler(void)
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    uint8_t checksum = (uint8_t)(VMC_CMD_VEND & 0x00FF) + VMC_VEND_FAILURE;
    uint8_t incoming_checksum;
    uint16_t temp;
    // Wait for 1 element in buffer
    // 1 checksum

    mdb_read(&temp, "109");
    incoming_checksum = (uint8_t)(temp & 0x00FF); // get rid of Mode bit if present
    if (checksum != incoming_checksum)
    {
        mdb_write(CSH_NAK, "37");
        return; // checksum mismatch, error
    }
    /* refund through server connection */

    mdb_write(CSH_ACK, "38"); // in case of success
    // MalfunctionError(); -- in case of failure, like unable to connect to server
    // Return state to SESSION IDLE
    m_csh_state = CSH_STATE_SESSION_IDLE;
    m_csh_poll_state = CSH_END_SESSION_POLL_STATE;

    MDB_LOG(LOG_INFO, "setting state to VEND_STATE_FAILED");
    m_vend_state = VEND_STATE_FAILED;
}

void MDB_Driver::vend_session_complete(void)
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    mdb_write(CSH_ACK, "41. vend_session_complete");
    m_csh_poll_state = CSH_END_SESSION_POLL_STATE;
}

void MDB_Driver::vend_cash_sale(void)
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    uint8_t i; // counter
    uint8_t checksum = (uint8_t)(VMC_CMD_VEND & 0x00FF) + VMC_VEND_CASH_SALE;
    uint8_t vend_data[5];
    uint16_t vend_temp;
    // Wait for 5 elements in buffer
    // 4 data + 1 checksum

    for (i = 0; i < 5; ++i)
    {
        mdb_read(&vend_temp, "108");
        vend_data[i] = (uint8_t)(vend_temp & 0x00FF); // get rid of Mode bit if present
    }
    checksum += calc_checksum(vend_data, 4);
    if (checksum != vend_data[4])
    {
        mdb_write(CSH_NAK, "34");
        return; // checksum mismatch, error
    }

    /* Cash sale implementation */

    mdb_write(CSH_ACK, "35");
}

void MDB_Driver::mdb_handle_reader()
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    uint8_t i; // counter
    uint8_t checksum = (uint8_t)(VMC_CMD_READER & 0x00FF);
    uint16_t reader_temp;
    uint8_t reader_data[2];

    // Store received data in array
    for (i = 0; i < 2; ++i)
    {
        mdb_read(&reader_temp, "102. mdb_handle_reader");
        reader_data[i] = (uint8_t)(reader_temp & 0x00FF); // get rid of Mode bit if present
    }

    // Calculate checksum
    checksum += calc_checksum(reader_data, 1);

    // Second element is an incoming checksum, compare it to the calculated one
    if (checksum != reader_data[1])
    {
        mdb_write(CSH_NAK, "25. mdb_handle_reader");
        MDB_LOG(LOG_ERR, "checksum failed, got %d instd of %d", reader_data[1], checksum);
        return; // checksum mismatch, error
    }

    // Look at Subcommand
    switch(reader_data[0])
    {
        case VMC_READER_DISABLE : disable(); break;
        case VMC_READER_ENABLE  : enable();    break;
        case VMC_READER_CANCEL  : cancelled(); break;
        default : break;
    }
}

void MDB_Driver::mdb_handle_expansion()
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    uint16_t readCmd;

    mdb_read(&readCmd, "101");
    switch(readCmd)
    {
        case VMC_EXPANSION_REQUEST_ID  : expansion_request_ID(); break;
        case VMC_EXPANSION_DIAGNOSTICS : expansion_diagnostics(); break;
            // Actually I never got VMC_EXPANSION_DIAGNOSTICS subcommand, so whatever
        default : break;
    }
    // Not yet implemented

}

void MDB_Driver::vend_success_handler(void)
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    uint8_t i; // counter
    uint8_t checksum = (uint8_t)(VMC_CMD_VEND & 0x00FF) + VMC_VEND_SUCCESS;
    uint8_t vend_data[3];
    uint16_t vend_temp;

    for (i = 0; i < 3; ++i)
    {
        mdb_read(&vend_temp, "11. vend_success_handler");
        vend_data[i] = (uint8_t)(vend_temp & 0x00FF); // get rid of Mode bit if present
    }
    /* here goes another check-check with a server */
    mdb_write(CSH_ACK, "42. vend_success_handler");

    // Return state to SESSION IDLE
    m_csh_state = CSH_STATE_SESSION_IDLE;
    m_vend_state = VEND_STATE_SUCCEEDED;
    m_csh_poll_state = CSH_END_SESSION_POLL_STATE;
}

/*
 * Internal functions for MDB_ReaderHandler()
 */
void MDB_Driver::disable(void)
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    m_csh_state = CSH_STATE_DISABLED;
    mdb_write(CSH_ACK ,  "3. disable");
}

void MDB_Driver::enable(void)
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    if (m_csh_state == CSH_STATE_DISABLED)
    {
        m_csh_state = CSH_STATE_ENABLED;
    }

    m_csh_poll_state = CSH_ACK_POLL_STATE;
    mdb_write(CSH_ACK,  "4. enable");
}

void MDB_Driver::cancelled(void)
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    if (m_csh_state != CSH_STATE_ENABLED)
        return;
    mdb_write(CSH_CANCELLED ,  "1");
    mdb_write(CSH_CANCELLED | CSH_ACK,  "2"); // Checksum with Mode bit
}

/*
 * Internal functions for MDB_ExpansionHandler()
 */
void MDB_Driver::expansion_request_ID(void)
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    uint16_t i; // counter
    uint8_t checksum = (uint8_t)(VMC_CMD_EXPANSION & 0x00FF) + VMC_EXPANSION_REQUEST_ID;
    uint16_t temp;
    uint8_t data[30];
    /*
     * Wait for incoming 29 data elements + 1 checksum (30 total)
     * Store the data by the following indexes:
     *  0,  1,  2 -- Manufacturer Code (3 elements)
     *  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14 -- Serial Number (12 elements)
     * 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26 -- Model  Number (12 elements)
     * 27, 28 -- Software Version (2 elements)
     * 29 -- Checksum (1 element)
     */

    // Store data
    for (i = 0; i < 30; ++i)
    {
        mdb_read(&temp, "100");
        data[i] = (uint8_t)(temp & 0x00FF); // get rid of Mode bit if present
    }

    // Calculate checksum
    checksum += calc_checksum(data, 29);
    // Second element is an incoming checksum, compare it to the calculated one
    if (checksum != data[29])
    {
        mdb_write(CSH_NAK, "6");
        MDB_LOG(LOG_ERR, "Wrong checksum, got %d instd of %d", data[29], checksum);
        return; // checksum mismatch, error
    }
    // Respond with our own data
    send_peripheral_ID();
}

void MDB_Driver::expansion_diagnostics(void)
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    uint8_t checksum = (uint8_t)(VMC_CMD_EXPANSION & 0x00FF) + VMC_EXPANSION_DIAGNOSTICS;
    mdb_write(CSH_ACK , "5");

}

void MDB_Driver::set_user_funds(uint16_t user_funds)
{
    MDB_LOG(LOG_INFO, "%s: %d", __PRETTY_FUNCTION__, user_funds);
    m_user_funds = user_funds;
}

void MDB_Driver::start_vending_state()
{

    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    m_vend_state = VEND_STATE_WAITING_SELECTION;

//    m_csh_poll_state = CSH_BEGIN_SESSION_POLL_STATE;
    m_csh_poll_state = CSH_ACK_POLL_STATE;
//    m_csh_poll_state = CSH_BEGIN_SESSION_POLL_STATE;

    if ( m_simulation_mode != NO_SIMULATION)
    {
        MDB_LOG(LOG_INFO, "in simulation state");
        std::thread select_thread = std::thread([](MDB_Driver* driver)
                                                {
                                                    MDB_LOG(LOG_INFO, "sleep_for 10 sec to simulate selection");
                                                    std::this_thread::sleep_for(std::chrono::seconds(10));
                                                    if (driver->m_simulation_mode == NO_SELECTION_SCENARIO)
                                                    {
                                                        MDB_LOG(LOG_INFO, "setting vend state to VEND_STATE_FAILED");
                                                        driver->m_vend_state = VEND_STATE_FAILED;
                                                    }
                                                    else
                                                    {
                                                        MDB_LOG(LOG_INFO, "setting vend state to VEND_STATE_WAITING_APPROVAL");
                                                        driver->m_vend_state = VEND_STATE_WAITING_APPROVAL;
                                                        driver->m_item_price = 500;
                                                    }
                                                }, this);
        select_thread.detach();
    }
}

void MDB_Driver::end_vending_state()
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    m_csh_poll_state = CSH_END_SESSION_POLL_STATE;
    m_vend_state = VEND_STATE_IDLE;
}

uint16_t MDB_Driver::get_item_price()
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    MDB_LOG(LOG_INFO, "Item price: %d", m_item_price);
    return m_item_price;
}

uint16_t MDB_Driver::get_max_price()
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    if (m_simulation_mode != NO_SIMULATION)
    {
        return 2000;
    }
    else
    {
        return m_vmc_prices.maxPrice;
    }
}

void MDB_Driver::vend_denied()
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    mdb_write(CSH_VEND_DENIED, "36");
    mdb_write(CSH_VEND_DENIED | CSH_ACK); // Checksum with Mode bit set
    m_csh_poll_state = CSH_END_SESSION_POLL_STATE;
    m_vend_state = VEND_STATE_DENIED;
}


uint16_t MDB_Driver::get_amount_of_items()
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    return m_vend_amount;
}

// ADD

void MDB_Driver::trigger_begin_session()
{
    MDB_LOG(LOG_INFO, __PRETTY_FUNCTION__ );
    m_csh_poll_state = CSH_BEGIN_SESSION_POLL_STATE;
}
