// ==============================================================
// RTL generated by Vivado(TM) HLS - High-Level Synthesis from C, C++ and SystemC
// Version: 2016.2
// Copyright (C) 1986-2016 Xilinx, Inc. All Rights Reserved.
// 
// ===========================================================

#ifndef _fir_HH_
#define _fir_HH_

#include "systemc.h"
#include "AESL_pkg.h"


namespace ap_rtl {

struct fir : public sc_module {
    // Port declarations 17
    sc_in_clk ap_clk;
    sc_in< sc_logic > ap_rst;
    sc_in< sc_logic > ap_start;
    sc_out< sc_logic > ap_done;
    sc_out< sc_logic > ap_idle;
    sc_out< sc_logic > ap_ready;
    sc_out< sc_lv<5> > result_address0;
    sc_out< sc_logic > result_ce0;
    sc_out< sc_logic > result_we0;
    sc_out< sc_lv<32> > result_d0;
    sc_in< sc_lv<32> > result_q0;
    sc_out< sc_lv<5> > result_address1;
    sc_out< sc_logic > result_ce1;
    sc_out< sc_logic > result_we1;
    sc_out< sc_lv<32> > result_d1;
    sc_in< sc_lv<32> > result_q1;
    sc_in< sc_lv<32> > n;


    // Module declarations
    fir(sc_module_name name);
    SC_HAS_PROCESS(fir);

    ~fir();

    sc_trace_file* mVcdFile;

    ofstream mHdltvinHandle;
    ofstream mHdltvoutHandle;
    sc_signal< sc_lv<3> > ap_CS_fsm;
    sc_signal< sc_logic > ap_sig_cseq_ST_st1_fsm_0;
    sc_signal< bool > ap_sig_18;
    sc_signal< sc_logic > ap_sig_cseq_ST_st2_fsm_1;
    sc_signal< bool > ap_sig_46;
    sc_signal< sc_lv<1> > exitcond_fu_97_p2;
    sc_signal< sc_lv<5> > i_1_fu_137_p2;
    sc_signal< sc_logic > ap_sig_cseq_ST_st3_fsm_2;
    sc_signal< bool > ap_sig_61;
    sc_signal< sc_lv<5> > i_reg_85;
    sc_signal< sc_lv<64> > sum_cast_fu_109_p1;
    sc_signal< sc_lv<64> > sum3_cast_fu_120_p1;
    sc_signal< sc_lv<64> > tmp_fu_125_p1;
    sc_signal< sc_lv<32> > tmp_1_fu_130_p2;
    sc_signal< sc_lv<5> > sum_fu_103_p2;
    sc_signal< sc_lv<5> > sum3_fu_114_p2;
    sc_signal< sc_lv<3> > ap_NS_fsm;
    static const sc_logic ap_const_logic_1;
    static const sc_logic ap_const_logic_0;
    static const sc_lv<3> ap_ST_st1_fsm_0;
    static const sc_lv<3> ap_ST_st2_fsm_1;
    static const sc_lv<3> ap_ST_st3_fsm_2;
    static const sc_lv<32> ap_const_lv32_0;
    static const sc_lv<1> ap_const_lv1_1;
    static const sc_lv<32> ap_const_lv32_1;
    static const sc_lv<1> ap_const_lv1_0;
    static const sc_lv<32> ap_const_lv32_2;
    static const sc_lv<5> ap_const_lv5_2;
    static const sc_lv<64> ap_const_lv64_0;
    static const sc_lv<64> ap_const_lv64_1;
    static const sc_lv<5> ap_const_lv5_14;
    static const sc_lv<5> ap_const_lv5_1F;
    static const sc_lv<5> ap_const_lv5_1E;
    static const sc_lv<5> ap_const_lv5_1;
    // Thread declarations
    void thread_ap_clk_no_reset_();
    void thread_ap_done();
    void thread_ap_idle();
    void thread_ap_ready();
    void thread_ap_sig_18();
    void thread_ap_sig_46();
    void thread_ap_sig_61();
    void thread_ap_sig_cseq_ST_st1_fsm_0();
    void thread_ap_sig_cseq_ST_st2_fsm_1();
    void thread_ap_sig_cseq_ST_st3_fsm_2();
    void thread_exitcond_fu_97_p2();
    void thread_i_1_fu_137_p2();
    void thread_result_address0();
    void thread_result_address1();
    void thread_result_ce0();
    void thread_result_ce1();
    void thread_result_d0();
    void thread_result_d1();
    void thread_result_we0();
    void thread_result_we1();
    void thread_sum3_cast_fu_120_p1();
    void thread_sum3_fu_114_p2();
    void thread_sum_cast_fu_109_p1();
    void thread_sum_fu_103_p2();
    void thread_tmp_1_fu_130_p2();
    void thread_tmp_fu_125_p1();
    void thread_ap_NS_fsm();
    void thread_hdltv_gen();
};

}

using namespace ap_rtl;

#endif