// digita PI control
inline void calPIControl(int& motorCommand1, int& motorCommand2, float dAngleRate_RadPerS1, float dAngleRate_RadPerS2) {
    // desired encoder pulses per sampling period
    int dAngleRate_CountPerTs1 = 0;
    int dAngleRate_CountPerTs2 = 0;

    // convert dAngleRate_RadPerS? to dAngleRate_CountPerTs?
    static const float Conv1 = (GEAR_RATIO_1 * RES_ENC_1) / (2 * PI) * (T_S / (1000.0 * 1000));
    static const float Conv2 = (GEAR_RATIO_2 * RES_ENC_2) / (2 * PI) * (T_S / (1000.0 * 1000));
    dAngleRate_CountPerTs1 = dAngleRate_RadPerS1 * Conv1;
    dAngleRate_CountPerTs2 = dAngleRate_RadPerS2 * Conv2;

    // errors
    int err1 = dAngleRate_CountPerTs1 - angleRate_CountPerTs1;
    int err2 = dAngleRate_CountPerTs2 - angleRate_CountPerTs2;

    // accumulated errors
    static int accErr1 = 0;
    static int accErr2 = 0;
    accErr1 += err1;
    accErr2 += err2;

    motorCommand1 = Kp1 * err1 + Ki1 * accErr1;
    motorCommand2 = Kp2 * err2 + Ki2 * accErr2;
}

