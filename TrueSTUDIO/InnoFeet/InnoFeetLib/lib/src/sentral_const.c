

const struct _SENtralRegister SENtralRegister = {
// Results:
    /* QX */ 0x00,
    /* QY */ 0x04,
    /* QZ */ 0x08,
    /* QW */ 0x0C,
    /* QTime */ 0x10,
    /* MX */ 0x12,
    /* MY */ 0x14,
    /* MZ */ 0x16,
    /* MTime */ 0x18,
    /* AX */ 0x1A,
    /* AY */ 0x1C,
    /* AZ */ 0x1E,
    /* ATime */ 0x20,
    /* GX */ 0x22,
    /* GY */ 0x24,
    /* GZ */ 0x26,
    /* GTime */ 0x28,
    /* SentralStatus */ 0x37,
    /* AlgorithmStatus */ 0x38,
    /* PassThroughStatus */ 0x9E,
    /* EventStatus */ 0x35,
    /* RAMVersion */ 0x72,
    /* SensorStatus */ 0x36,
    /* ErrorRegister */ 0x50,
    /* HostControl */ 0x34,
    /* PassThroughControl */ 0xA0,
    /* ResetReq */ 0x9B,
    /* AlgorithmControl */ 0x54,
    /* EnableEvents */ 0x33,
    /* MagRate */ 0x55,
    /* AccelRate */ 0x56,
    /* GyroRate */ 0x57,
    /* QRateDivisor */ 0x32,
    /* ParamRequest */ 0x64,
    /* LoadParamBytes */ 0x60,
    /* LoadParamByte0 */ 0x60,
    /* LoadParamByte1 */ 0x61,
    /* LoadParamByte2 */ 0x62,
    /* LoadParamByte3 */ 0x63,
    /* RetrieveParamBytes */ 0x3B,
    /* RetrieveParamByte0 */ 0x3B,
    /* RetrieveParamByte1 */ 0x3C,
    /* RetrieveParamByte2 */ 0x3D,
    /* RetrieveParamByte3 */ 0x3E
};

const struct _SENtralBitFlags SENtralBitFlags = {
    //SentralStatus:
    /* EEPROM[0]  */            0b00000001,
    /* EEUploadDone[1]  */      0b00000010,
    /* EEUploadError[2]  */     0b00000100,
    /* Idle[3]  */              0b00001000,
    /* NoEEPROM[4]  */          0b00010000,
    //EventStatus & EnableEvents:
    /* CPURest[0]  */           0b00000001,
    /* Error[1]  */             0b00000010,
    /* QuaternionResult[2]  */  0b00000100,
    /* MagResult[3]  */         0b00001000,
    /* AccelResult[4]  */       0b00010000,
    /* GyroResult[5]  */        0b00100000,
    //HostControl:
    /* RunEnable[0]  */         0b00000001,
    //AlgorithmControl:
    /* StandbyEnable[0]  */     0b00000001,
    /* RawDataEnable[1]  */     0b00000010,
    /* HPRoutput[2]  */         0b00000100,
    /* ParamTransfer[7]  */     0b10000000,
    //ParamRequest & ParamAcknowledge:
    /* Parameter[0:6]  */       0b01111111,
    /* LoadOrSave[7]  */        0b10000000,
    /* ClearParam[0:7] */       0b00000000
};