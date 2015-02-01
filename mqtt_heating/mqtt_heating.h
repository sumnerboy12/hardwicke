
typedef struct {
  byte pin;
  byte state;
  byte published;
  long time;
} Pin;

void readDigital(Pin& pin);
void readAnalog(Pin& pin);
