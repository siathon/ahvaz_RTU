#include "main.h"

void update_header_footer(){
    show_date_time();
    show_battery();
    show_gsm_state(sim800.sim_registered);
    status_update("Running...");
}

void main_loop(){
    time_t now = time(NULL);
    check_arduino_data();
    check_power_state();
    Watchdog::get_instance().kick();

    if(last_data_timestamp != -1 && last_arduino_check != -1 && now >= (last_data_timestamp + 300) && now >= (last_arduino_check + 300)){
        arduino.printf("exit");
        is_arduino_parser_free = true;
        interface_warning = 1;
        interface_warning_sent = false;
    }

    if(last_data_timestamp != -1 && now >= (last_data_timestamp + 1800)){
        NVIC_SystemReset();
    }
    if(now < (last_data_timestamp + 300)){
        if(interface_warning == 1 && interface_warning_sent){
        interface_warning = 0;
    }
    }
    cnt++;
    if(on_battery){
        if(!one_time_gps_sent){
            send_loc_request_sms();
        }
        sim800.disable();
        if(now > last_lcd_op + 5){
            backlight = 0;
        }
    }
    else{
        one_time_gps_sent = false;
        if(cnt >= 5){
            ev_queue.call(update_header_footer);
            cnt = 0;
        }
        if(!manual_off){
            if(!backlight){
                show_sensor_data();
            }
            backlight = 1;
        }
        if(!sms_time_set){
            if((!time_set && now - last_set_time_try > 300) || (now - last_set_time_try > 3600)){
                get_time();
            }
        }
    }
    ev_queue.call_in(1000, main_loop);
}

int main(){
    // zero_eeprom();
    print_reset_reason();
    printf("\r\nFirmware version = %.1f\r\n", firmware_version);
    init_lcd();
    initialize_data();
    check_power_state();
    show_battery();
    show_gsm_state(false);
    draw_footer();
    status_update("Initializing");
    init_SD();
    Watchdog::get_instance().start();
    bd.init();
    load_config();
    show_device_id();
    status_update("Initializing.");

    logg("Resetting arduino...\r\n");
    logg("reset pin = 0\r\n");
    arduino_reset.output();
    arduino_reset = 0;
    wait_us(100000);
    logg("reset pin = 1\r\n");
    arduino_reset.input();
    wait_us(1000000);
    logg("Done.\r\n");


    Watchdog::get_instance().kick();
    status_update("Initializing. .");
    if(serial.readable()){
        while(serial.readable()){
            serial.getc();
        }
    }
    serial.attach(callback(&ser, &SerialHandler::rx));
    status_update("Initializing. . . .");
    // gps.attach(gps_rx);
    status_update("Initializing. . . . .");
    if(rs_menu.readable()){
        while(rs_menu.readable()){
            rs_menu.getc();
        }
    }
    rs_menu.attach(rs_menu_rx);
    status_update("Initializing. . . . . .");
    arduino.printf("exit");
    if(arduino.readable()){
        while(arduino.readable()){
            arduino.getc();
        }
    }
    arduino.attach(arduino_rx);
    status_update("Initializing. . . . . . .");
    
    
    Watchdog::get_instance().kick();
    show_sensor_data();
    get_time();

    show_date_time();
    status_update("Running...");

	ev_queue.call_in(100, blink);
    ev_queue.call_in(200, check_buttons);
    ev_queue.call_in(1000, main_loop);

    ev_queue.call(send_loc_request_sms);
    ev_queue.call(check_for_update);
    ev_queue.call_in(sd_log_interval, log_data_to_sd);
    ev_queue.call_in(data_post_interval, post_data);
    ev_queue.call_in(data_sms_interval, send_data_sms, false);
    ev_queue.call(check_for_sms);

	ev_queue.dispatch_forever();
}