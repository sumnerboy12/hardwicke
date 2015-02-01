typedef struct {
  // sent from remote nodes
  byte pin;
  int data;
  int battery;
  // filled in by the RF bridge
  byte nodeId;
  int rssi;
} I2CPacket;

