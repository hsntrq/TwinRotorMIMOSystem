unsigned long rpmtime;
float rpmfloat;
unsigned int rpm;
bool tooslow = 1;


void RPM () {
  rpmtime = TCNT1;
  TCNT1 = 0;
  tooslow = 0;
}


ISR(TIMER1_OVF_vect) {
  tooslow = 1;
}

void measure_RPM()
{
  delay(1000);
  if (tooslow == 1) {
    Serial.println("Too slow");
  }
  else {
    rpmfloat = 120 / (rpmtime/ 31250.00);
    Serial.println(rpmfloat);
  }
}