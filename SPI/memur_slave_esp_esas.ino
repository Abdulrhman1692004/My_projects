#include <ESP32SPISlave.h>

ESP32SPISlave slave;

static constexpr uint32_t BUFFER_SIZE {32};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

#define LED 2
void setup() {
    Serial.begin(115200);
    delay(2000);
    pinMode(LED, OUTPUT);
    // begin() after setting
    // HSPI = CS: 15, CLK: 14, MOSI: 13, MISO: 12 -> default
    // VSPI = CS:  5, CLK: 18, MOSI: 23, MISO: 19
    slave.setDataMode(SPI_MODE0);
    slave.begin();
    // slave.begin(VSPI);   // you can use VSPI like this

    // clear buffers
    memset(spi_slave_tx_buf, 0, BUFFER_SIZE);
    memset(spi_slave_rx_buf, 0, BUFFER_SIZE);
}

void loop() {
    // block until the transaction comes from master
    slave.wait(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);

    // if transaction has completed from master,
    // available() returns size of results of transaction,
    // and buffer is automatically updated
    char data;
    while (slave.available()) {
        // show received data
         Serial.print("Command Received: ");
         Serial.println(spi_slave_rx_buf[0]);
         data = spi_slave_rx_buf[0];
         Serial.print(data);
         slave.pop();
    }
    if(data == 1 )
    {
        Serial.println("Setting LED active HIGH ");
        digitalWrite(LED, HIGH);
    }
    else if(data == 0 )
    {
        Serial.println("Setting LED active LOW ");
        digitalWrite(LED, LOW);
    }
     Serial.println("");
}