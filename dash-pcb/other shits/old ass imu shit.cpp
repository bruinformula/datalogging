//put imu read sht here ig

// Read data for the adafruit 3463 if enough time has passed
void read_3463() {
  // If current time is past time for next reading, do reading
  if (micros() < next_imu_micros) return;

  // Storage for the 7 bytes that will be sent back by each communication
  uint8_t response[7];
  // Sum of shifted MSB and LSB
  short total;
  // Data scaled to a more understandable unit
  int converted_total_x;
  int converted_total_y;
  int converted_total_z;
  // Message that will be logged
  char log_message[128];

  // Get data from accelerometer output registers
  acc_mag.read(0x00, response, 7, false);

  if (!(response[0] & 0b111)) {
    log_pair("MSG", "ANR");
    prep_3463();
    next_imu_micros = micros() + 1000000;
    return;
  } else {
    // combine 6 bits MSB in response[1] with 8 bits LSB in [2]
    total = (int16_t)(((response[1] << 8) | response[2])) >> 2;
    // convert from arbitrary bit value to ug and log
    converted_total_x = ((int)total) * 488;

    total = (int16_t)(((response[3] << 8) | response[4])) >> 2;
    converted_total_y = ((int)total) * 488;

    total = (int16_t)(((response[5] << 8) | response[6])) >> 2;
    converted_total_z = ((int)total) * 488;

    // unit is ug, or one millionth of the acceleration due to gravity
    sprintf(log_message, "%i,%i,%i", converted_total_x, converted_total_y,
            converted_total_z);
    log_pair("ACC", log_message);
    tele_data_1B[0] = converted_total_x;
    tele_data_1B[1] = converted_total_y;
    tele_data_1B[2] = converted_total_z;
  }

  // Get data from gyroscope output registers
  gyr.read(0x00, response, 7, false);

  if (!(response[0] & 0b111)) {
    log_pair("MSG", "GNR");
    prep_3463();
    next_imu_micros = micros() + 1000000;
    return;
  } else {
    // combine 8 bits MSB in response[1] with 8 bits LSB in [2]
    total = (short)(((response[1] << 8) | response[2]));
    // convert from arbitrary bit value to mdps and log
    converted_total_x = ((int)total) * 125 / 16;

    total = (short)(((response[3] << 8) | response[4]));
    converted_total_y = ((int)total) * 125 / 16;

    total = (short)(((response[5] << 8) | response[6]));
    converted_total_z = ((int)total) * 125 / 16;

    // unit is mdps, or thousandths of a degree per second
    sprintf(log_message, "%i,%i,%i", converted_total_x, converted_total_y,
            converted_total_z);
    log_pair("GYR", log_message);
    tele_data_1B[3] = converted_total_x;
    tele_data_1B[4] = converted_total_y;
    tele_data_1B[5] = converted_total_z;
  }

  // Set time for next reading
  next_imu_micros = max(micros(), next_imu_micros + IMU_MICROS_INCR);
}

// Initialize the adafruit 3463
void prep_3463() {
  next_imu_micros = micros() + IMU_MICROS_INCR;
  uint8_t result_byte;

  // This register is the id of the device, which should always be 0xC7
  acc_mag.read(ACC_MAG_WHOAMI_REG, &result_byte, false);

  if (result_byte != 0xC7) {
    // wrong or no id, print error
    log_file.print("ERR,am whoami,");
    log_file.println(result_byte, HEX);
  } else {
    log_file.println("MSG,acc_mag found");
  }

  // This register is the id of the device, which should always be 0b11010111
  acc_mag.read(GYR_WHOAMI_REG, &result_byte, false);

  if (result_byte != 0b11010111) {
    log_file.print("ERR,am whoami,");
    log_file.println(result_byte, HEX);
  } else {
    Serial.println("GYR sensor found");
    log_file.println("MSG,gyr found");
  }

  // byte to be written to device
  uint8_t msg;

  // set accelerometer range to +/- 4g
  // see datasheet for explanation of registers
  msg = 0b00000001;
  acc_mag.write(ACC_MAG_XYZ_DATA_CFG_REG, msg, false);
  // set accelerometer to active mode, max data rate
  msg = 0b00000001;
  acc_mag.write(ACC_MAG_CTRL_REG_1, msg, false);

  // set gyro range to +/- 250dps
  msg = 0b00000011;
  gyr.write(GYR_CTRL_REG0, msg, false);

  // set to 800Hz, active
  msg = 0b00000011;
  gyr.write(GYR_CTRL_REG1, msg, false);
}
