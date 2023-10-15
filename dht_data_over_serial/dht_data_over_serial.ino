#include <dht_nonblocking.h>
#define DHT_SENSOR_TYPE DHT_TYPE_11

static const int DHT_PIN = 2;
DHT_nonblocking dht_sensor(DHT_PIN, DHT_SENSOR_TYPE);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  float temp;
  float hum;

  if(measure_env(&temp, &hum) == true) {
    Serial.print(hum);
    Serial.print(", ");
    Serial.print(temp);
    Serial.print("\n");
  }
}

static bool measure_env(float *temp, float *hum) {
  static unsigned long measurement_timestamp = millis();

  if(millis() - measurement_timestamp > 3000ul) {
    if(dht_sensor.measure(temp, hum) == true) {
      measurement_timestamp = millis();
      return (true);
    }
  }
  return(false);
}
