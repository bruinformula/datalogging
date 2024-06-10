//telemetry shits

// send string for wireless telementry
void wireless_tele() {
  if (micros() > next_realtime_tele_micros) {
    char data_string[150] = {0};
    // ACCX,ACCY,ACCZ|GYRX,GYRY,GYRZ|A1|
    // engineSpeed(RPM)|engineLoad(%)|throttle(%)|
    // intakeTemp(C)|coolantTemp(C)|
    // lambda1|manifold_pressure(kPa)|fan1(bool)|
    // logFileName|targetO2|DBTDC|INJDUTY|VBAT|FPUMP|FPR
    // 0: ACC, 1: GYR, 2: EGT, 3: ENGSPD, 4: ENGLD, 5: TPS,
    // 6: IAT, 7: CLT, 8: O2, 9: MAP, 10: FAN, 11: LOGNAME, 12: TO2, 13: DBTDC,
    // 14: INJDUTY, 15: VBAT, 16: FPUMP, 17: FPR, 18: EGT
    sprintf(data_string,
            "BFR%d,%d,%d|%d,%d,%d|%d|%d,%d|%d,%d|%d,%d|%d|%d|%d|%d,%d|%d|%s|%d|"
            "%d|%d|%d,%d|%d|%d|\n",
            tele_data_1B[0], tele_data_1B[1], tele_data_1B[2], tele_data_1B[3],
            tele_data_1B[4], tele_data_1B[5], tele_data_1B[6],
            tele_data_2B[0][0], tele_data_2B[0][1], tele_data_2B[1][0],
            tele_data_2B[1][1], tele_data_2B[2][0], tele_data_2B[2][1],
            tele_data_1B[7], tele_data_1B[8], tele_data_1B[9],
            tele_data_2B[3][0], tele_data_2B[3][1], tele_data_fan1, log_name,
            tele_data_1B[12], tele_data_1B[10], tele_data_1B[13],
            tele_data_2B[4][0], tele_data_2B[4][1], tele_data_fpump,
            tele_data_1B[11]);
    Serial8.print(data_string);

    next_realtime_tele_micros = micros() + REALTIME_TELE_MICROS_INCR;
  }
}
