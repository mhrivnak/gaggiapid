// include the library code:
#include <LiquidCrystal.h>
#include <dht_nonblocking.h>
#define DHT_SENSOR_TYPE DHT_TYPE_11

static const int DHT_SENSOR_PIN = 2;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

void setup() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
}

void loop() {
  float temperature;
  float humidity;
  if( measure( &temperature, &humidity ) == true )
  {
     lcd.setCursor(0, 0);
     lcd.print("Temp: ");
     lcd.print(temperature);
     lcd.print(" C");
     lcd.setCursor(0, 1);
     lcd.print("Humd: ");
     lcd.print(humidity);
     lcd.print("%");
  }
}

static bool measure( float *temperature, float *humidity )
{
  static unsigned long measurement_timestamp = millis( );

  /* Measure once every four seconds. */
  if( millis( ) - measurement_timestamp > 3000ul )
  {
    if( dht_sensor.measure( temperature, humidity ) == true )
    {
      measurement_timestamp = millis( );
      return( true );
    }
  }

  return( false );
}
