/*********************************************************************************************************************
* MSPM0G3507 Opensource Library 鍗筹紙MSPM0G3507 寮€婧愬簱锛夋槸涓€涓熀浜庡畼鏂?SDK 鎺ュ彛鐨勭涓夋柟寮€婧愬簱
* Copyright (c) 2022 SEEKFREE 閫愰绉戞妧
* 
* 鏈枃浠舵槸 MSPM0G3507 寮€婧愬簱鐨勪竴閮ㄥ�?
* 
* MSPM0G3507 寮€婧愬�?鏄厤璐硅蒋�?
* 鎮ㄥ彲浠ユ牴鎹嚜鐢辫蒋浠跺熀閲戜細鍙戝竷�?GPL锛圙NU General Public License锛屽�?GNU閫氱敤鍏叡璁稿彲璇侊級鐨勬潯�?
* �?GPL 鐨勭�?鐗堬紙鍗?GPL3.0锛夋垨锛堟偍閫夋嫨鐨勶級浠讳綍鍚庢潵鐨勭増鏈紝閲嶆柊鍙戝竷�?鎴栦慨鏀瑰畠
* 
* 鏈紑婧愬簱鐨勫彂甯冩槸甯屾湜瀹冭兘鍙戞尌浣滅敤锛屼絾骞舵湭瀵瑰叾浣滀换浣曠殑淇濊�?
* 鐢氳嚦娌℃湁闅愬惈鐨勯€傞攢鎬ф垨閫傚悎鐗瑰畾鐢ㄩ€旂殑淇濊瘉
* 鏇村缁嗚妭璇峰弬瑙?GPL
* 
* 鎮ㄥ簲璇ュ湪鏀跺埌鏈紑婧愬簱鐨勫悓鏃舵敹鍒颁竴浠?GPL 鐨勫壇鏈?
* 濡傛灉娌℃湁锛岃鍙傞槄<https://www.gnu.org/licenses/>
* 
* 棰濆娉ㄦ槑�?
* 鏈紑婧愬簱浣跨�?GPL3.0 寮€婧愯鍙瘉鍗忚�?浠ヤ笂璁稿彲鐢虫槑涓鸿瘧鏂囩増鏈?
* 璁稿彲鐢虫槑鑻辨枃鐗堝湪 libraries/doc 鏂囦欢澶逛笅�?GPL3_permission_statement.txt 鏂囦欢涓?
* 璁稿彲璇佸壇鏈�?libraries 鏂囦欢澶逛笅 鍗宠鏂囦欢澶逛笅�?LICENSE 鏂囦�?
* 娆㈣繋鍚勪綅浣跨敤骞朵紶鎾湰绋嬪簭 浣嗕慨鏀瑰唴瀹规椂蹇呴』淇濈暀閫愰绉戞妧鐨勭増鏉冨０鏄庯紙鍗虫湰澹版槑锛?
* 
* 鏂囦欢鍚嶇�?         mian
* 鍏徃鍚嶇�?         鎴愰兘閫愰绉戞妧鏈夐檺鍏�?
* 鐗堟湰淇℃伅          鏌ョ�?libraries/doc 鏂囦欢澶瑰唴 version 鏂囦�?鐗堟湰璇存槑
* 寮€鍙戠幆澧?         MDK 5.37
* 閫傜敤骞冲彴          MSPM0G3507
* 搴楅摵閾炬帴          https://seekfree.taobao.com/
********************************************************************************************************************/

#include "zf_common_headfile.h"
// 鎵撳紑鏂扮殑宸ョ▼鎴栬€呭伐绋嬬Щ鍔ㄤ簡浣嶇疆鍔″繀鎵ц浠ヤ笅鎿嶄綔
// 绗竴姝?鍏抽棴涓婇潰鎵€鏈夋墦寮€鐨勬枃�?
// 绗簩姝?project->clean  绛夊緟涓嬫柟杩涘害鏉¤蛋�?

// ?��緥绋嬫槸寮€婧愬簱绌哄伐绋?鍙敤浣滅Щ妞嶆垨鑰呮祴璇曞悇绫诲唴澶栬
// 鏈緥绋嬫槸寮€婧愬簱绌哄伐�?鍙敤浣滅Щ妞嶆垨鑰呮祴璇曞悇绫诲唴澶栬
// 鏈緥绋嬫槸寮€婧愬簱绌哄伐�?鍙敤浣滅Щ妞嶆垨鑰呮祴璇曞悇绫诲唴澶栬

// **************************** 浠ｇ爜鍖哄煙 ****************************

uint32 pid_tuning_time_ms = 0;

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);   // 鏃堕挓閰嶇疆鍙婄郴缁熷垵濮嬪�?鍔″繀淇濈暀>
    debug_init();					// 璋冭瘯涓插彛淇℃伅鍒濆�?
	// 姝ゅ缂栧啓鐢ㄦ埛浠ｇ爜 渚嬪澶栬鍒濆鍖栦唬鐮佺�?
    system_delay_ms(300);           
    
// 猸愶�?鍒濆鍖?OLED 灞忓�?
    oled_init();
    oled_clear();

    key_init(5);					
    beep_init(5);
    
    // 猸愶�?鏍戣帗娲句覆鍙ｉ€氫俊鍒濆�?(寤鸿鍚庣画鎶婃尝鐗圭巼鏀逛綆鐐癸紝姣斿 115200)
    comm_init();					
    task3_ball_init();            // Local Task3 state only; no Pi link.
    
    adc_init(ADC0_CH7_A22, ADC_12BIT); 
    adc_init(ADC1_CH5_B18, ADC_12BIT); 
    adc_capture_init();
    Motor_init();                // 鐢垫満鍒濆鍖?            
    
    Init_ICM42688();
    Filter_Init();                       // 婊ゆ尝鍣ㄥ垵濮嬪�?
    
    // 鈿狅�?娴嬭瘯闃舵寮虹儓寤鸿鍏堟妸闄€铻轰华鏍″噯娉ㄩ噴鎺夛紝鍚﹀垯濡傛灉娌℃斁骞充細瀵艰嚧姝绘満
    IMU_calibration(); 
    
    // 鍚姩瀹氭椂鍣紝浼犲�?NULL 閬垮紑搴曞眰鍐椾綑鍥炶皟
    pit_ms_init(PIT_TIM_A0, 5, NULL, NULL);
    pit_ms_init(PIT_TIM_A1, 5, NULL, NULL);
    
    interrupt_set_priority(TIMA0_INT_IRQn, 1); // 涓柇浼樺厛�?0-7 瓒婁綆瓒婇珮
    interrupt_set_priority(TIMA1_INT_IRQn, 0); // 璁剧疆瀹氭椂鍣ˋ1涓柇浼樺厛绾т负1
    
    beep_set_time(100);
    assign_value();                 // 鍒濆鍖栧弬鏁板�?
    gpio_init(B26, GPO, GPIO_LOW, GPO_PUSH_PULL); 

    interrupt_global_enable(0); // 寮€鍚叏灞€鎬婚椄�?

while(true)
    {
        // 1. 璇荤數姹犵數�?(绾�?ADC0锛屽摢鎬曡涓柇鎵撴柇涔熸鏃犲奖�?
        battery_voltage = adc_mean_filter_convert(ADC0_CH7_A22, 10) * 0.0089388f;

        // 2. 猸愶�?澶勭悊鎸夐敭涓庤彍鍗曢€昏緫 (鍗充娇閲岄潰�?300ms 寤舵椂锛屼篃涓嶄細褰卞搷 5ms 涓柇閲岀殑搴曠洏鎺у埗)
        menu_control();

        // 3. 猸愶�?鍒锋�?OLED 灞忓�?UI (鏋佸叾鑰楁椂锛屽繀椤绘斁鍦ㄨ繖閲屾參鎱㈢敾)
        show_ui();
        
        // 4. UART2 通信
        // Task10 -> PID 调参协议（CSV + SET/STATUS）
        // Raspberry Pi task communication is intentionally disconnected.
        if (task_number == TASK_PID_OVAL_TRACK) {
            uint32 primask = interrupt_global_disable();
                float input_snapshot = line_error_filtered;
                float pwm_snapshot = Duty_dS_applied;
                float kp_snapshot = PIDK_YA.Kp;
                float ki_snapshot = PIDK_YA.Ki;
                float kd_snapshot = PIDK_YA.Kd;
            interrupt_global_enable(primask);

            pid_tuning_process_command();
            send_pid_tuning_status(pid_tuning_time_ms,
                                   0.0f,
                                   input_snapshot,
                                   pwm_snapshot,
                                   -input_snapshot,
                                   kp_snapshot,
                                   ki_snapshot,
                                   kd_snapshot);
            pid_tuning_time_ms += 10;
        } else {
            pid_tuning_time_ms = 0;

        }
        system_delay_ms(10); // 浼戞伅涓€涓嬶紝闃叉鐤媯鍒峰�?
    }
}


