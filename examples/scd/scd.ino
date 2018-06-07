#include <Wire.h>

#include <scdsensor.h>

ScdSensor scd;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(1000); // let serial console settle

  int ret = scd.init();
  if (!ret) {
      Serial.print("init(): success\n");
  } else {
      Serial.print("init(): failed, ret = ");
      Serial.println(ret);
  }
}

void loop() {
  if (scd.checkDataReady()) {
    Serial.print("\n");
    int ret = scd.readSample();
    if (!ret) {
        Serial.print("CO2: ");
        Serial.print(scd.getCO2());
        Serial.print("ppm | ");

        Serial.print("Temp: ");
        Serial.print(scd.getTemperature());
        Serial.print("C | ");

        Serial.print("RH: ");
        Serial.print(scd.getHumidity());
        Serial.print("%\n");
    } else {
        Serial.print("Error in readSample(), ret = ");
        Serial.println(ret);
    }
  } else {
    // indicate polling
    Serial.print(".");
  }
  delay(500);
}
