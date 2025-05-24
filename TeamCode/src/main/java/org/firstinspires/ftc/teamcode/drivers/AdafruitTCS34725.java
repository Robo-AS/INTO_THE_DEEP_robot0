package org.firstinspires.ftc.teamcode.drivers;


import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;


@I2cDeviceType
@DeviceProperties(name = "TCS34725 Adafruit Color Sensor", xmlTag = "TCS34725")
public class AdafruitTCS34725 extends I2cDeviceSynchDevice<I2cDeviceSynch>{

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x29);

    public AdafruitTCS34725(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }


    public enum Register {
        FIRST(0x0F),
//        ATIME(0x01),
//        WTIME(0x03),
//        AILTL(0x04),
//        AILTH(0x05),
//        AIHTL(0x06),
//        AIHTH(0x07),
//        PERS(0x0C),
//        CONFIG(0x0D),
//        CONTROL(0x0F),
        ID(0x12),
        STATUS(0x13),
        CDATAL(0x14),
        CDATAH(0x15),
        RDATAL(0x16),
        RDATAH(0x17),
        GDATAL(0x18),
        GDATAH(0x19),
        BDATAL(0x1A),
        BDATAH(0x1B),
        LAST(BDATAH.bVal);


        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }


    protected void setOptimalReadWindow() {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }


    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return "TCS34725 Adafruit Color Sensor";
    }


    protected void writeShort(final Register reg, short value) {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    protected byte readByte(Register reg) {
        return deviceClient.read8(0x80 | reg.bVal);
    }

    public short getManufacturerIDRaw() {
        return readByte(Register.ID);
    }
}
