#include"userInterface.h"
void showDistance(void) {
    char buffer [10];
    lcdClear();
    lcdSetPos(0, 0);
    lcdWriteStrC("Distance: [cm/uS]");
    lcdSetPos(0, 1);
    sprintf(buffer, "%3.3f", readDistance());
    lcdWriteStrC(buffer);
}

void showTempLight(void) {
    char buffer [10];
    lcdClear();
    lcdSetPos(0, 0);
    sprintf(buffer, "temp: %1.3f", readTempF());
    lcdWriteStrC(buffer);
    lcdSetPos(0, 1);
    sprintf(buffer, "light: %d", readLight());
    lcdWriteStrC(buffer);
}

void showAccelerometerVal(void) {
    char buffer [10];
    lcdClear();

    lcdWriteStrC("Aclmtr values:");
    lcdSetPos(0, 1);
    sprintf(buffer, "%1.2f", single_axis_measure(X_AXIS, iteration_point));
    lcdWriteStrC(buffer);
    lcdWriteChar(' ');
    sprintf(buffer, "%1.2f", single_axis_measure(Y_AXIS, iteration_point));
    lcdWriteStrC(buffer);
    lcdWriteChar(' ');
    sprintf(buffer, "%1.2f", single_axis_measure(Z_AXIS, iteration_point));
    lcdWriteStrC(buffer);
}

