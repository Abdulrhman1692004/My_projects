//Slave
//SPI tanımlama
#include "helper.h"
#include <ESP32SPISlave.h>
ESP32SPISlave slave;
static constexpr size_t BUFFER_SIZE = 8;
static constexpr size_t QUEUE_SIZE = 1;
uint8_t tx_buf[BUFFER_SIZE] {1,2,3,4,5,6,7,8};
uint8_t rx_buf[BUFFER_SIZE] {0, 0, 0, 0, 0, 0, 0, 0};


#include <Wire.h>
#include "MLX90641_API.h"
#include "MLX9064X_I2C_Driver.h"
#define TA_SHIFT 8

const byte MLX90641_address = 0x33; 
uint16_t eeMLX90641[832]; 
uint16_t MLX90641Frame[242]; 
float MLX90641To[192];
paramsMLX90641 MLX90641;
float get_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void set_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y, float f);
void get_adjacents_1d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void get_adjacents_2d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
float cubicInterpolate(float p[], float x);
float bicubicInterpolate(float p[], float x, float y);
void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols, float *dest, uint8_t dest_rows, uint8_t dest_cols);
float dest_2d[36 * 48]; 
boolean isConnected() {
  Wire.beginTransmission((uint8_t)MLX90641_address);
  if (Wire.endTransmission() != 0) { // 0: meaning success
    return (false);                  //Sensor did not ACK
  }
  return (true);
}
float get_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y) {
  if (x < 0)
    x = 0;
  if (y < 0)
    y = 0;
  if (x >= cols)
    x = cols - 1;
  if (y >= rows)
    y = rows - 1;
  return p[y * cols + x];
}
void set_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y, float f) {
  if ((x < 0) || (x >= cols))
    return;
  if ((y < 0) || (y >= rows))
    return;
  p[y * cols + x] = f;
}
void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols, float *dest, uint8_t dest_rows, uint8_t dest_cols) {
  float mu_x = (src_cols - 1.0) / (dest_cols - 1.0);
  float mu_y = (src_rows - 1.0) / (dest_rows - 1.0);
  float adj_2d[16]; 
  for (uint8_t y_idx = 0; y_idx < dest_rows; y_idx++) {
    for (uint8_t x_idx = 0; x_idx < dest_cols; x_idx++) {
      float x = x_idx * mu_x;
      float y = y_idx * mu_y;
      get_adjacents_2d(src, adj_2d, src_rows, src_cols, x, y);
      float frac_x = x - (int)x;
      float frac_y = y - (int)y; 
      float out = bicubicInterpolate(adj_2d, frac_x, frac_y);
      set_point(dest, dest_rows, dest_cols, x_idx, y_idx, out);
    }
  }
}
float cubicInterpolate(float p[], float x) {
  float r = p[1] + (0.5 * x * (p[2] - p[0] + x * (2.0 * p[0] - 5.0 * p[1] + 4.0 * p[2] - p[3] + x * (3.0 * (p[1] - p[2]) + p[3] - p[0]))));
  return r;
}
// p is a 16-point 4x4 array of the 2 rows & columns left/right/above/below
float bicubicInterpolate(float p[], float x, float y) {
  float arr[4] = {0, 0, 0, 0};
  arr[0] = cubicInterpolate(p + 0, x);
  arr[1] = cubicInterpolate(p + 4, x);
  arr[2] = cubicInterpolate(p + 8, x);
  arr[3] = cubicInterpolate(p + 12, x);
  return cubicInterpolate(arr, y);
}
void get_adjacents_1d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y) {
  dest[0] = get_point(src, rows, cols, x - 1, y);
  dest[1] = get_point(src, rows, cols, x, y);
  dest[2] = get_point(src, rows, cols, x + 1, y);
  dest[3] = get_point(src, rows, cols, x + 2, y);
}
void get_adjacents_2d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y) {
  float arr[4];
  for (int8_t delta_y = -1; delta_y < 3; delta_y++) { // -1, 0, 1, 2
    float *row = dest + 4 * (delta_y + 1); // index into each chunk of 4
    for (int8_t delta_x = -1; delta_x < 3; delta_x++) { // -1, 0, 1, 2
      row[delta_x + 1] = get_point(src, rows, cols, x + delta_x, y + delta_y);
    }
  }
}
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000); 
  while(!Serial);
  if(isConnected() == false) Serial.println("MLX90641 not detected at default I2C address. Check wiring!");
  int status;
  status = MLX90641_DumpEE(MLX90641_address, eeMLX90641);
  if(status != 0) Serial.println("Failed to load system parameters!");  
  status = MLX90641_ExtractParameters(eeMLX90641, &MLX90641);
  if(status != 0) Serial.println("Parameter extraction failed!");
  MLX90641_SetRefreshRate(MLX90641_address, 0x04);
  
  //SPI baslangic
    
  slave.setDataMode(SPI_MODE0);   // default: SPI_MODE0
  slave.setQueueSize(QUEUE_SIZE); // default: 1, requres 2 in this example

  // begin() after setting
  slave.begin();  // default: HSPI (please refer README for pin assignments)

  Serial.println("start spi slave");
}
void loop() {
    for(byte x = 0; x < 2; x++){
      int status = MLX90641_GetFrameData(MLX90641_address, MLX90641Frame);
      float vdd = MLX90641_GetVdd(MLX90641Frame, &MLX90641);
      float Ta = MLX90641_GetTa(MLX90641Frame, &MLX90641);
      float tr = Ta - TA_SHIFT;
      float emissivity = 0.95;
      MLX90641_CalculateTo(MLX90641Frame, &MLX90641, emissivity, tr, MLX90641To);
      interpolate_image(MLX90641To, 12, 16, dest_2d, 24, 32);      
    }
    String thermalFrame = "[";
    for (int i = 1; i < 192*4 + 1; i++) { // Starts at 1 so that (i % 8) will be 0 at end of 8 pixels and newline will be added
      thermalFrame += dest_2d[i - 1];
      if ( i != 192*4 ){
        thermalFrame += ", ";
        if ( i % 32 == 0 ) thermalFrame += "\n";
      }
    }
    thermalFrame += "]";    
   char tf[thermalFrame.length()+1];
    thermalFrame.toCharArray(tf, thermalFrame.length()+1); //string adı tf olan bir char array

    uint32_t tflenght = sizeof(tf);

  //SPI kod başlangıcı

  // if no transaction is in flight and all results are handled, queue new transactions
    if (slave.hasTransactionsCompletedAndAllResultsHandled()) {
        // initialize tx/rx buffers
        Serial.println("initialize tx/rx buffers");
        initializeBuffers(tx_buf, rx_buf, BUFFER_SIZE, 0);

        // queue transaction and trigger it right now
        Serial.println("execute transaction in the background");
        slave.queue(tx_buf, rx_buf, BUFFER_SIZE);
        slave.trigger();

        Serial.println("wait for the completion of the queued transactions...");
    }

    // you can do some other stuff here
    // NOTE: you can't touch dma_tx/rx_buf because it's in-flight in the background

    // if all transactions are completed and all results are ready, handle results
    if (slave.hasTransactionsCompletedAndAllResultsReady(QUEUE_SIZE)) {
        // process received data from slave
        Serial.println("all queued transactions completed. start verifying received data from slave");

        // get the oldeest transfer result
        size_t received_bytes = slave.numBytesReceived();

        // verify and dump difference with received data
        // NOTE: we need only 1st results (received_bytes[0])
        if (verifyAndDumpDifference("slave", tx_buf, BUFFER_SIZE, "master", rx_buf, received_bytes)) {
            Serial.println("successfully received expected data from master");
        } else {
            Serial.println("unexpected difference found between master/slave data");
        }
    }
}

