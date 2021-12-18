#define ENA 5
#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9
#define ENB 10

double motorSpeedA = 255;
double motorSpeedB = 255;

double controlSpeed;

void thrustAUp()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, motorSpeedA);
}

void thrustADown()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, motorSpeedA);
}

void thrustBUp()
{
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, motorSpeedB);
}

void thrustBDown()
{
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, motorSpeedB);
}

void OffA() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void OffB() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
