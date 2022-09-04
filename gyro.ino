

#include <CurieIMU.h>
#include <MadgwickAHRS.h>

#include <Wire.h>
#include <U8g2lib.h>

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void setup() {
  Serial.begin(9600);

  // SSD1306 
  pinMode(9, OUTPUT);
  digitalWrite(9, 0);  // default output in I2C mode for the SSD1306 test shield: set the i2c adr to 0
  u8g2.begin();

  u8g2.clearBuffer();                    // バッファのクリア
  u8g2.setFont(u8g2_font_t0_15_tf );    // フォント設定
  u8g2.drawStr(0, 10, "Hello World!");   // バッファに文字列を格納
  u8g2.sendBuffer();                     // バッファの内容で画面を更新

  delay(3000);

  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);

  // Set the accelerometer range to 2 g
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
}

void loop() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;

    update_display(roll,pitch,heading);
  }
}

void update_display(float roll, float pitch, float yaw){
  int cursor_x = 0,cursor_y = 10;
  int font_height = 15;
  char buff[7];
  
  u8g2.clearBuffer();                    // バッファのクリア
  u8g2.setFont(u8g2_font_t0_15_tf );    // フォント設定

  dtostrf(roll,6,2,buff);
  cursor_x += u8g2.drawStr(0,cursor_y,"roll : ");   // バッファに文字列を格納
  u8g2.drawStr(cursor_x,cursor_y,buff); 
  cursor_y += font_height;
  
  cursor_x = 0;
  dtostrf(pitch,6,2,buff);
  cursor_x += u8g2.drawStr(0,cursor_y,"pitch: ");   // バッファに文字列を格納
  u8g2.drawStr(cursor_x,cursor_y,buff); 
  cursor_y += font_height;
  
  cursor_x = 0;
  dtostrf(yaw,6,2,buff);
  cursor_x += u8g2.drawStr(0,cursor_y,"yaw  : ");   // バッファに文字列を格納
  u8g2.drawStr(cursor_x,cursor_y,buff); 
  
  u8g2.sendBuffer();                     // バッファの内容で画面を更新
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2 g range
  // -2 g maps to a raw value of -32768
  // +2 g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}
