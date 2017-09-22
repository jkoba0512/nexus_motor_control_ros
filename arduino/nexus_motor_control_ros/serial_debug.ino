
#if defined(SERIAL_DEBUG)
// mode: direct input of motor command
void modeDirectMotorCommandInput(int& motorCommand1, int& motorCommand2) {
  if (Serial.available() > 0) {
    switch (Serial.read()) {
      case 'a':
        motorCommand1 += 30;
        break;
      case 's':
        motorCommand1 = 0;
        break;
      case 'd':
        motorCommand1 -= 30;
        break;
      case 'w':
        motorCommand1 = 255;
        break;
      case 'x':
        motorCommand1 = -255;
        break;
      case 'j':
        motorCommand2 += 30;
        break;
      case 'k':
        motorCommand2 = 0;
        break;
      case 'l':
        motorCommand2 -= 30;
        break;
      case 'i':
        motorCommand2 = 255;
        break;
      case ',':
        motorCommand2 = -255;
        break;
      default:
        ;
    }
  }

  // output
  Serial.print("e:");
  Serial.print(encCountPerTs1);
  Serial.print(",");
  Serial.print(encCountPerTs2);
  Serial.print(", ");
  Serial.print("c:");
  Serial.print(motorCommand1);
  Serial.print(",");
  Serial.print(motorCommand2);
  Serial.println();
}

#endif  // SERIAL_DEBUG

