package org.firstinspires.ftc.teamcode.onbotjava;

import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.TypeConversion;
import java.util.Arrays;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;

@I2cDeviceType
@DeviceProperties(name = "MPU6050 Gyroscope v4", description = "MPU6050 6-axis accelerometer", xmlTag = "MPU6050v4")
public class MPU6050Gyro extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    public double getAcceleration_x_g() {
        updateReadingsIfNecessary();
        return accelerometerReadingX_g;
    }
    public double getAcceleration_y_g() {
        updateReadingsIfNecessary();
        return accelerometerReadingY_g;
    }
    public double getAcceleration_z_g() {
        updateReadingsIfNecessary();
        return alignmentX_deg;
    }
    
    
    public double getAlignment_x_deg() {
        updateReadingsIfNecessary();
        return alignmentX_deg;
    }
    public double getAlignment_y_deg() {
        updateReadingsIfNecessary();
        return alignmentY_deg;
    }
    public double getAlignment_z_deg() {
        updateReadingsIfNecessary();
        return alignmentZ_deg;
    }
    
    
    public double getRotation_x_degPerSec() {
        updateReadingsIfNecessary();
        return rotationReadingX_degPerSec;
    }
    public double getRotation_y_degPerSec() {
        updateReadingsIfNecessary();
        return rotationReadingY_degPerSec;
    }
    public double getRotation_z_degPerSec() {
        updateReadingsIfNecessary();
        return rotationReadingZ_degPerSec;
    }


    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x68);

    long reading_millis=0;
    double accelerometerReadingX_g,accelerometerReadingY_g, accelerometerReadingZ_g;
    // Find alignment in degrees (-180..180)
    double alignmentX_deg, alignmentY_deg, alignmentZ_deg;
    
    double rotationReadingX_degPerSec, rotationReadingY_degPerSec, rotationReadingZ_degPerSec;


    // This is assembled from
    //  Example Idea: https://github.com/FIRST-Tech-Challenge/WikiSupport/blob/master/SampleOpModes/java/i2cExample/MCP9808.java#L133
    //  Arduino example: https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
    //  Official Register Addresses: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
    public enum Register {
        RESET(0x6B),
        ACCEL_READ_ALL(0x3B), // 6 bytes, 2 bytes for each of X, Y and Z
        ACCEL_READ_X(0x3B),
        ACCEL_READ_Y(0x3D),
        ACCEL_READ_Z(0x3F),
        GYRO_READ_ALL(0x43), // 6 bytes, 2 bytes for each of X, Y and Z
        GYRO_READ_X(0x43),
        GYRO_READ_Y(0x45),
        GYRO_READ_Z(0x47);

        int addr;
        Register(int addr) {this.addr = addr;}
    }

    protected void updateReadingsIfNecessary() {
        long now = System.currentTimeMillis();
        if ( now - reading_millis > 20 ) {
            reading_millis = now;
            byte sixBytes[] = deviceClient.read(Register.ACCEL_READ_ALL.addr, 6);
            accelerometerReadingX_g = TypeConversion.byteArrayToShort(Arrays.copyOfRange(sixBytes, 0, 2))/16384.0;
            accelerometerReadingY_g = TypeConversion.byteArrayToShort(Arrays.copyOfRange(sixBytes, 2, 4))/16384.0;
            accelerometerReadingZ_g = TypeConversion.byteArrayToShort(Arrays.copyOfRange(sixBytes, 4, 6))/16384.0;

            alignmentX_deg = 180.0*accelerometerReadingX_g;
            alignmentY_deg = 180.0*accelerometerReadingY_g;
            alignmentY_deg = 180.0*accelerometerReadingZ_g;

            sixBytes = deviceClient.read(Register.GYRO_READ_ALL.addr, 6);
            rotationReadingX_degPerSec = TypeConversion.byteArrayToShort(Arrays.copyOfRange(sixBytes, 0, 2))/131.0;
            rotationReadingY_degPerSec = TypeConversion.byteArrayToShort(Arrays.copyOfRange(sixBytes, 2, 4))/131.0;
            rotationReadingZ_degPerSec = TypeConversion.byteArrayToShort(Arrays.copyOfRange(sixBytes, 4, 6))/131.0;
        }
    }

    public MPU6050Gyro(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        deviceClient.engage();
    }

    protected synchronized boolean doInitialize() {
        deviceClient.write(Register.RESET.addr, TypeConversion.shortToByteArray((short)0));

        return true;
    }


    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.HiTechnic;
    }

    @Override
    public String getDeviceName()
    {
        return "MPU6050 Gyro v2";
    }

    public String getTelemetryString() {
        return String.format("acc=[%+.1f, %+.1f, %+.1f]g algn=[%+.1f, %+.1f, %+.1f]deg rot=[%.0f, %.0f, %.0f]dps",
            getAcceleration_x_g(), getAcceleration_y_g(), getAcceleration_z_g(),
            getAlignment_x_deg(), getAlignment_y_deg(), getAlignment_z_deg(),
            getRotation_x_degPerSec(), getRotation_y_degPerSec(), getRotation_z_degPerSec());
    }
}
