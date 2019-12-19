#include "FrSky.h"
#include "FrSkySportSensorFcs.h"
#include "FrSkySportSensorVario.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportTelemetry.h"
#include "FrSkySportSensorAss.h"
#include "SoftwareSerial.h"
#include "FrSkySportSensorRpm.h"

// Initialize sensor types
FrSkySportSensorAss ass;
FrSkySportSensorFcs fcs;
FrSkySportSensorRpm rpm;
FrSkySportSensorVario vario;
FrSkySportTelemetry telemetry;


void BifrostSetup() {
   telemetry.begin(FrSkySportSingleWireSerial::SERIAL_3, &ass, &fcs, &rpm, &vario);
}

void BifrostSend(float as, uint8_t id, float v, bool soc, uint8_t sel, float ext) {
   // Set data to fields: FrSky field (Custom field)
   // Change sensor names on transmitter to match custom field
   ass.setData(as); // Airspeed (Speed)
   fcs.setData(id, v); // Current (TestID), Volage (Vbatt)
   rpm.setData(0, sel, soc); // RPM (), Temp 1 (Ctrl Sel), Temp 2 (Soc Eng)
   vario.setData(0, ext); // Altitude (), Vertical Speed (Excite Eng)

  // Send the telemetry data, note that the data will only be sent for sensors
  // that are being polled at given moment
  telemetry.send();
}
