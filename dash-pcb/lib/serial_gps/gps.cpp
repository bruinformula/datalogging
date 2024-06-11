// GPS libraries
#include <SoftwareSerial.h>
#include <TimeLib.h>
#include <TinyGPS.h>

// GPS objects
// TinyGPS gps;
// String newNMEA = "";
// String GPGGANMEA = "";
// String GPRMCNMEA = "";

void read_GPS() {
  // Parse GPS data if received
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    newNMEA += c;       // Build NMEA string
    if (gps.encode(c))  // True if new sentence
    {
      // Parse for NMEA Variants
      int GPGGAindex = newNMEA.lastIndexOf("$GPGGA");
      int GPGGAend = newNMEA.indexOf("$", GPGGAindex + 1);
      if (GPGGAindex != -1) {
        GPGGANMEA = newNMEA.substring(GPGGAindex, GPGGAend - 1);
      }
      int GPRMCindex = newNMEA.lastIndexOf("$GPRMC");
      int GPRMCend = newNMEA.indexOf("$", GPRMCindex + 1);
      if (GPRMCindex != -1) {
        GPRMCNMEA = newNMEA.substring(GPRMCindex, GPRMCend - 1);
      }
      newNMEA = "";
    }
    if (newNMEA.length() > 4000) {
      newNMEA.remove(0, 2000);  // Removes characters to keep newNMEA from
                                // growing when there's no fix
    }
  }

  if (micros() > next_gps_micros) {
    // Prepare variable to turn into epoch time
    int year;
    uint8_t month, day, hour, minutes, seconds, hundredths;
    unsigned long int age;
    gps.crack_datetime(&year, &month, &day, &hour, &minutes, &seconds,
                       &hundredths, &age);
    TimeElements t;
    t.Year = year - 1970;
    t.Month = month;
    t.Day = day;
    t.Hour = hour;
    t.Minute = minutes;
    t.Second = seconds;
    if (GPGGANMEA != "" && GPRMCNMEA != "") {
      // Log NMEA string and epoch time
      log_pair("GPGGA NMEA", GPGGANMEA);
      log_pair("GPRMC NMEA", GPRMCNMEA);
      log_pair("Time", makeTime(t));
    }

    next_gps_micros = micros() + GPS_MICROS_INCR;
  }
}
