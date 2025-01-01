//
// Created by zeevm on 21/03/2023.
//

#include <jni.h>
#include <syslog.h>
#include "MDB_Driver.h"

JNIEnv* g_env = nullptr;
//extern "C"
//JNIEXPORT void JNICALL
//Java_com_adytech_mdb_1vending_MDB_1Vending_native_1MDB_1slave_1func(JNIEnv *env,
//                                                                               jobject thiz) {
//    MDB_Driver::get_instance().mdb_slave_func();
//}
extern "C"
JNIEXPORT jint JNICALL
Java_com_adytech_mdb_1vending_MDB_1Vending_native_1MDB_1get_1max_1price(JNIEnv *env,
                                                                                   jobject thiz) {
    return (jint)MDB_Driver::get_instance().get_max_price();
}
extern "C"
JNIEXPORT void JNICALL
Java_com_adytech_mdb_1vending_MDB_1Vending_native_1MDB_1set_1user_1funds(JNIEnv *env,
                                                                                    jobject thiz,
                                                                                    jint user_funds) {
    MDB_Driver::get_instance().set_user_funds((int)user_funds);
}
extern "C"
JNIEXPORT void JNICALL
Java_com_adytech_mdb_1vending_MDB_1Vending_native_1MDB_1start_1vending_1state(
        JNIEnv *env, jobject thiz) {
    g_env = env;
    MDB_Driver::get_instance().start_vending_state();
}

extern "C"
JNIEXPORT void JNICALL
Java_com_adytech_mdb_1vending_MDB_1Vending_native_1MDB_1slave_1func(JNIEnv *env, jobject thiz) {
    MDB_Driver::get_instance().mdb_slave_func();
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_adytech_mdb_1vending_MDB_1Vending_native_1MDB_1get_1vending_1state(JNIEnv *env,
                                                                            jobject thiz) {
    return (jint)MDB_Driver::get_instance().MDB_get_vend_state();
}
extern "C"
JNIEXPORT jint JNICALL
Java_com_adytech_mdb_1vending_MDB_1Vending_native_1MDB_1get_1item_1price(JNIEnv *env, jobject thiz) {
    return (jint)MDB_Driver::get_instance().get_item_price();
}

extern "C"
JNIEXPORT void JNICALL
Java_com_adytech_mdb_1vending_MDB_1Vending_native_1MDB_1end_1vend_1state(JNIEnv *env,
                                                                         jobject thiz
) {
    MDB_Driver::get_instance().end_vending_state();
}

extern "C"
JNIEXPORT void JNICALL
Java_com_adytech_mdb_1vending_MDB_1Vending_native_1MDB_1vend_1approval(JNIEnv *env, jobject thiz,
                                                                       jboolean approved) {
    MDB_Driver::get_instance().vend_approval((bool)approved);
}

extern "C"
JNIEXPORT void JNICALL
Java_com_adytech_mdb_1vending_MDB_1Vending_native_1MDB_1set_1simulation_1mode(JNIEnv *env, jobject thiz,
                                                                       jint simulation_mode) {
    MDB_Driver::get_instance().set_simulation_mode((int)simulation_mode);
}


extern "C"
JNIEXPORT void JNICALL
Java_com_adytech_mdb_1vending_MDB_1Vending_native_1MDB_1trigger_1begin_1session(JNIEnv *env,
                                                                          jobject thiz) {
    MDB_Driver::get_instance().trigger_begin_session();
}


