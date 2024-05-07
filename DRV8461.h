// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/

/// \file DRV8461.h
///
/// This is the main header file for the DRV8461 library,
/// a library for controlling the DRV8461 stepper motor driver.
///
/// For more information about this library, see:
///
///   https://github.com/pololu/DRV8461-arduino
///
/// That is the main repository for this library.

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <bitset>

/// Addresses of control and status registers.
// SEE https://rb.gy/ykep8x for more information
enum class DRV8461RegAddr: uint8_t
{
  FAULT = 0x00,
  
  // DIAGNOSTICS 
  DIAG1 = 0x01,
  DIAG2 = 0x02,
  DIAG3 = 0x03,
  
  // CONTROL REGS used to configure the device
  CTRL1  = 0x04,
  CTRL2  = 0x05,
  CTRL3  = 0x06,
  CTRL4  = 0x07,
  CTRL5  = 0x08,
  CTRL6  = 0x09,
  CTRL7  = 0x0A,
  CTRL8  = 0x0B,
  CTRL9  = 0x0C,
  CTRL10 = 0x0D,
  CTRL11 = 0x0E,
  CTRL12 = 0x0F,
  CTRL13 = 0x10,
  
    // INDEX REGS
  INDEX1 = 0x11, // CUR_A_POS[7:0] (8 bits)
  INDEX2 = 0x12, // CUR_A_POS sign (bit 7)
  INDEX3 = 0x13, // CUR_B_POS[7:0] (8 bits)
  INDEX4 = 0x14, // CUR_B_POS sign (bit 7)
  INDEX5 = 0x15, // 

    // CUSTOM CONTROL REGS
  CUSTOM_CTRL_1 = 0x16,
  CUSTOM_CTRL_2 = 0x17,
  CUSTOM_CTRL_3 = 0x18,
  CUSTOM_CTRL_4 = 0x19,
  CUSTOM_CTRL_5 = 0x1A,
  CUSTOM_CTRL_6 = 0x1B,
  CUSTOM_CTRL_7 = 0x1C,
  CUSTOM_CTRL_8 = 0x1D,
  CUSTOM_CTRL_9 = 0x1E,

    // ATQ REGS
  ATQ_CTRL_1  = 0x1F,
  ATQ_CTRL_2  = 0x20,
  ATQ_CTRL_3  = 0x21,
  ATQ_CTRL_4  = 0x22,
  ATQ_CTRL_5  = 0x23,
  ATQ_CTRL_6  = 0x24,
  ATQ_CTRL_7  = 0x25,
  ATQ_CTRL_8  = 0x26,
  ATQ_CTRL_9  = 0x27,
  ATQ_CTRL_10 = 0x28,
  ATQ_CTRL_11 = 0x29,
  ATQ_CTRL_12 = 0x2A,
  ATQ_CTRL_13 = 0x2B,
  ATQ_CTRL_14 = 0x2C,
  ATQ_CTRL_15 = 0x2D,
  ATQ_CTRL_16 = 0x2E,
  ATQ_CTRL_17 = 0x2F,
  ATQ_CTRL_18 = 0x30,

    // SS CONTROL REGS
  SS_CTRL_1 = 0x31,
  SS_CTRL_2 = 0x32,
  SS_CTRL_3 = 0x33,
  SS_CTRL_4 = 0x34,
  SS_CTRL_5 = 0x35,

};


/// This class provides low-level functions for reading and writing from the SPI
/// interface of a DRV8461 stepper motor controller IC.
///
/// Most users should use the HighPowerStepperDriver class, which provides a
/// higher-level interface, instead of this class.
class DRV8461SPI
{
public: 
  /// Configures this object to use the specified pin as a chip select pin.
  ///
  /// You must use a chip select pin; the DRV8461 requires it.
  void setChipSelectPin(uint8_t pin)
  {
    csPin = pin;
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
  }

  /// Reads the register at the given address and returns its raw value.
  uint8_t readReg(uint8_t address)
  {
    // Arduino out / DRV8434 in: First byte contains read/write bit and register
    // address; second byte is unused.
    // Arduino in / DRV8434 out: First byte contains status; second byte
    // contains data in register being read.

    selectChip();
            lastStatus = transfer((0x20 | (address & 0b11111)) << 1);
    uint8_t data       = transfer(0);
    deselectChip();
    return data;
  }

  /// Reads the register at the given address and returns its raw value.
  uint16_t readReg(DRV8461RegAddr address)
  {
    return readReg((uint8_t)address);
  }

  /// Writes the specified value to a register.
  uint8_t writeReg(uint8_t address, uint8_t value)
  {
    // Arduino out / DRV8434 in: First byte contains read/write bit and register
    // address; second byte contains data to write to register.
    // Arduino in / DRV8434 out: First byte contains status; second byte
    // contains old (existing) data in register being written to.

    selectChip();
            lastStatus = transfer((address & 0b11111) << 1);
    uint8_t oldData    = transfer(value);
    // The CS line must go low after writing for the value to actually take
    // effect.
    deselectChip();
    return oldData;
  }

    /// Writes the specified value to a register.
  void writeReg(DRV8461RegAddr address, uint8_t value)
  {
    writeReg((uint8_t)address, value);
  }

private: 

  SPISettings settings = SPISettings(500000, MSBFIRST, SPI_MODE1);

  uint8_t transfer(uint8_t value)
  {
    return SPI.transfer(value);
  }

  void selectChip()
  {
    digitalWrite(csPin, LOW);
    SPI.beginTransaction(settings);
  }

  void deselectChip()
  {
   SPI.endTransaction();
   digitalWrite(csPin, HIGH);
  }

  uint8_t csPin;

public: 

  /// The status reported by the driver during the last read or write.  This
  /// status is the same as that which would be returned by reading the FAULT
  /// register with DRV8461::readFault(), except the upper two bits are always
  /// 1.
  uint8_t lastStatus = 0;
};


/// Bits that are set in the return value of readFault() to indicate warning and
/// fault conditions.
///
/// See the DRV8461 datasheet for detailed descriptions of these conditions.
enum class DRV8461FaultBit: uint8_t
{

  FAULT     = 7,   // Fault indication (0 when nFAULT pin is high, 1 when nFAULT pin is low)
  SPI_ERROR = 6,   /// SPI protocol error (latched)
  UVLO      = 5,   /// Supply undervoltage lockout fault
  CPUV      = 4,   /// Charge pump undervoltage fault
  OCP       = 3,   /// Overcurrent fault
  STL       = 2,   /// Motor stall
  TF        = 1,   /// Overtemperature warning or shutdown
  OL        = 0,   /// Open load

};

/// Bits that are set in the return value of readDiag1() to indicate warning and
/// fault conditions.
///
/// See the DRV8461 datasheet for detailed descriptions of these conditions.
enum class DRV8461Diag1Bit: uint8_t
{
  OCP_LS2_B = 7,   /// Overcurrent fault on low-side FET of half bridge 2 in BOUT
  OCP_HS2_B = 6,   /// Overcurrent fault on high-side FET of half bridge 2 in BOUT
  OCP_LS1_B = 5,   /// Overcurrent fault on low-side FET of half bridge 1 in BOUT
  OCP_HS1_B = 4,   /// Overcurrent fault on high-side FET of half bridge 1 in BOUT
  OCP_LS2_A = 3,   /// Overcurrent fault on low-side FET of half bridge 2 in AOUT
  OCP_HS2_A = 2,   /// Overcurrent fault on high-side FET of half bridge 2 in AOUT
  OCP_LS1_A = 1,   /// Overcurrent fault on low-side FET of half bridge 1 in AOUT
  OCP_HS1_A = 0,   /// Overcurrent fault on high-side FET of half bridge 1 in AOUT

};

/// Bits that are set in the return value of readDiag2() to indicate warning and
/// fault conditions.
///
/// See the DRV8461 datasheet for detailed descriptions of these conditions.
enum class DRV8461Diag2Bit: uint8_t
{
  
  STSL       = 7,   /// standstill power saving mode
  OTW        = 6,   /// Overtemperature warning
  OTS        = 5,   /// Overtemperature shutdown
  STL_LRN_OK = 4,   /// Stall detection learning successful
  STALL      = 3,   /// When this bit is 1b, it indicates motor is stalled.
  LRN_DONE   = 2,   // When this bit is 1b, it indicates auto torque learning is successful.
  OL_B       = 1,   /// Open load on BOUT
  OL_A       = 0,   /// Open load on AOUT

};

enum class DRV8461Diag3Bit: uint8_t
{
                //RSVD_7 = 7, // Reserved
  NHOME    = 6,  // When this bit is '1', it indicates indexer is at a position other than home position.
  CNT_OFLW = 5,  // When this bit is '1', it indicates ATQ_CNT is more than ATQ_UL
  CNT_UFLW = 4, // When this bit is '1', it indicates ATQ_CNT is less than ATQ_LL
                //RSVD_3 = 3, // Reserved
  NPOR     = 2,  // 0b = Indicates a prior VCC UVLO event, 1b = Indicates that the NPOR bit has been cleared by a CLR_FLT or nSLEEP reset pulse input after a VCC UVLO event
                //RSVD_1_0 = 2, // Reserved
};

/// Possible arguments to setDecayMode() using the DECAY bits in the CTRL1
enum class DRV8461DecayMode: uint8_t
{
  Slow                   = 0b000,
  IncSlowDecMixed30      = 0b001,
  IncSlowDecMixed60      = 0b010,
  IncSlowDecFast         = 0b011,
  Mixed30                = 0b100,
  Mixed60                = 0b101,
  SmartTuneDynamicDecay  = 0b110,
  SmartTuneRippleControl = 0b111,
};

/// Possible arguments to setStepMode().
enum class DRV8461StepMode: uint8_t
{
  MicroStep1_100 = 0b0000,   /// Full step with 100% current
  MicroStep1     = 0b0001,   /// Full step with 71% current
  MicroStep2_NC  = 0b0010,   /// Non-circular 1/2 step
  MicroStep2     = 0b0011,   /// Circular 1/2 step
  MicroStep4     = 0b0100,
  MicroStep8     = 0b0101,
  MicroStep16    = 0b0110,
  MicroStep32    = 0b0111,
  MicroStep64    = 0b1000,
  MicroStep128   = 0b1001,
  MicroStep256   = 0b1010,
};

/// This class provides high-level functions for controlling a DRV8461 stepper
/// motor driver.
class DRV8461
{
public: 
  /// The default constructor.
  DRV8461()
  {
                    // All settings set to power-on defaults
    ctrl1  = 0x0F;  //binary 0000 1111
    ctrl2  = 0x06;  //binary 0000 0110
    ctrl3  = 0x38;  //binary 0011 1000
    ctrl4  = 0x49;  //binary 0100 1001
    ctrl5  = 0x03;  //binary 0000 0011
    ctrl6  = 0x20;  //binary 0000 0011
    ctrl7  = 0xFF;  //binary 1111 1111
    ctrl8  = 0x0F;  //binary 0000 1111
    ctrl9  = 0x10;  //binary 0001 0000
    ctrl10 = 0x80;  //binary 1000 0000
    ctrl11 = 0xFF;  //binary 1111 1111
    ctrl12 = 0x20;  //binary 0000 0000
    ctrl13 = 0x58;  //binary 0000 0000
  }

  /// Configures this object to use the specified pin as a chip select pin.
  /// You must use a chip select pin; the DRV8711 requires it.
  void setChipSelectPin(uint8_t pin)
  {
    driver.setChipSelectPin(pin);
  }

  /// Changes all of the driver's settings back to their default values.
  ///
  /// It is good to call this near the beginning of your program to ensure that
  /// there are no settings left over from an earlier time that might affect the
  /// operation of the driver.
  void resetSettings()
  {
    ctrl1  = 0x0F;  //binary 0000 1111
    ctrl2  = 0x06;  //binary 0000 0110
    ctrl3  = 0x38;  //binary 0011 1000
    ctrl4  = 0x49;  //binary 0100 1001
    ctrl5  = 0x03;  //binary 0000 0011
    ctrl6  = 0x20;  //binary 0000 0011
    ctrl7  = 0xFF;  //binary 1111 1111
    ctrl8  = 0x0F;  //binary 0000 1111
    ctrl9  = 0x10;  //binary 0001 0000
    ctrl10 = 0x80;  //binary 1000 0000
    ctrl11 = 0xFF;  //binary 1111 1111
    ctrl12 = 0x20;  //binary 0000 0000
    ctrl13 = 0x58;  //binary 0000 0000
    applySettings();
  }

  /// Reads back the SPI configuration registers from the device and verifies
  /// that they are equal to the cached copies stored in this class.
  ///
  /// This can be used to verify that the driver is powered on and has not lost
  /// them due to a power failure.  The STATUS register is not verified because
  /// it does not contain any driver settings.
  ///
  /// @return 1 if the settings from the device match the cached copies, 0 if
  /// they do not.
  bool verifySettings()
  {
    return driver.readReg(DRV8461RegAddr::CTRL1)  == ctrl1 &&
           driver.readReg(DRV8461RegAddr::CTRL2)  == ctrl2 &&
           driver.readReg(DRV8461RegAddr::CTRL3)  == ctrl3 &&
           driver.readReg(DRV8461RegAddr::CTRL4)  == ctrl4 &&
           driver.readReg(DRV8461RegAddr::CTRL5)  == ctrl5 &&
           driver.readReg(DRV8461RegAddr::CTRL6)  == ctrl6 &&
           driver.readReg(DRV8461RegAddr::CTRL7)  == ctrl7 &&
           driver.readReg(DRV8461RegAddr::CTRL8)  == ctrl8 &&
           driver.readReg(DRV8461RegAddr::CTRL9)  == ctrl9 &&
           driver.readReg(DRV8461RegAddr::CTRL10) == ctrl10 &&
           driver.readReg(DRV8461RegAddr::CTRL11) == ctrl11 &&
           driver.readReg(DRV8461RegAddr::CTRL12) == ctrl12 &&
           driver.readReg(DRV8461RegAddr::CTRL13) == ctrl13;
  }

  /// Re-writes the cached settings stored in this class to the device.
  ///
  /// You should not normally need to call this function because settings are
  /// written to the device whenever they are changed.  However, if
  /// verifySettings() returns false (due to a power interruption, for
  /// instance), then you could use applySettings() to get the device's settings
  /// back into the desired state.
  void applySettings()
  {
    writeCachedReg(DRV8461RegAddr::CTRL2);
    writeCachedReg(DRV8461RegAddr::CTRL3);
    writeCachedReg(DRV8461RegAddr::CTRL4);
    writeCachedReg(DRV8461RegAddr::CTRL5);
    writeCachedReg(DRV8461RegAddr::CTRL6);
    writeCachedReg(DRV8461RegAddr::CTRL7);
    writeCachedReg(DRV8461RegAddr::CTRL8);
    writeCachedReg(DRV8461RegAddr::CTRL9);
    writeCachedReg(DRV8461RegAddr::CTRL10);
    writeCachedReg(DRV8461RegAddr::CTRL11);
    writeCachedReg(DRV8461RegAddr::CTRL12);
    writeCachedReg(DRV8461RegAddr::CTRL13);

    // CTRL1 is written last because it contains the EN_OUT bit, and we want to
    // try to have all the other settings correct first.
    writeCachedReg(DRV8461RegAddr::CTRL1);
  }

  /// Daz: I've modified the below to work with an 8bit regiter CTRL11 and convert 1-100% to 1-256
  
  /// Sets the driver's current scalar (TRQ_DAC), which scales the full current
  /// limit (as set by VREF) by the specified percentage. The available settings
  /// are multiples of 6.25%.
  ///
  /// This function takes an integer, and if the desired current limit is not
  /// available, it generally tries to pick the closest current limit that is
  /// lower than the desired one (although the lowest possible setting is
  /// 6.25%). However, it will round up if the next setting is no more than
  /// 0.75% higher; this allows you to specify 43.75% by passing a value of 43,
  /// for example.
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// // This sets TRQ_DAC to 37.5% (the closest setting lower than 42%):
  /// sd.setCurrentPercent(42);
  ///
  /// // This sets TRQ_DAC to 43.75% (rounding 43 up by 0.75% to 43.75%):
  /// sd.setCurrentPercent(43);
  ///
  /// // This also sets TRQ_DAC to 43.75%; even though the argument is truncated
  /// // to an integer (43), that is then rounded up by 0.75% to 43.75%:
  /// sd.setCurrentPercent(43.75);
  /// ~~~
  void setCurrentPercent(uint8_t percent)
  {
    if (percent > 100) { percent = 100; }
    if (percent < 1) { percent = 1; }

    uint8_t td     = ((uint8_t)percent * 255 + 99) / 100;  // convert 1-100% to 1-256
            ctrl11 = td;
    writeCachedReg(DRV8461RegAddr::CTRL11);
  }

  /// Sets the driver's current scalar (TRQ_DAC) to produce the specified scaled
  /// current limit in milliamps. In order to calculate the correct value for
  /// TRQ_DAC, this function also needs to know the full current limit set by
  /// VREF (i.e. what the current limit is when the scaling is set to 100%).
  /// This is specified by the optional `fullCurrent` argument, which defaults
  /// to 2000 milliamps (2 A).
  ///
  /// If the desired current limit is not
  /// available, this function tries to pick the closest current limit that is
  /// lower than the desired one (although the lowest possible setting is 6.25%
  /// of the full current limit).
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// // This specifies that we want a scaled current limit of 1200 mA and that
  /// // VREF is set to produce a full current limit of 1500 mA. TRQ_DAC will be
  /// // set to 75%, which will produce a 1125 mA scaled current limit.
  /// sd.setCurrentMilliamps(1200, 1500);
  /// ~~~
  void setCurrentMilliamps(uint16_t current, uint16_t fullCurrent = 2000)
  {
    if (fullCurrent > 4000) { fullCurrent = 4000; }
    if (current > fullCurrent) { current = fullCurrent; }

    uint8_t td = (current * 16 / fullCurrent);  // convert 0-fullCurrent to 0-16
    if      (td == 0) { td = 1; }               // restrict to 1-8 bits
    //td      = 16 - td;                          // convert 1-16 to 15-0 (15 = 6.25%, 0 = 100%)
    ctrl11 = td;
    writeCachedReg(DRV8461RegAddr::CTRL11);
  }

  /// Enables the driver (EN_OUT = 1).
  void enableDriver()
  {
    ctrl1 |= (1 << 7);
    writeCachedReg(DRV8461RegAddr::CTRL1);
  }

  /// Disables the driver (EN_OUT = 0).
  void disableDriver()
  {
    ctrl1 &= ~(1 << 7);
    writeCachedReg(DRV8461RegAddr::CTRL1);
  }

  /// Sets the driver's decay mode (DECAY).
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// sd.setDecayMode(DRV8461DecayMode::SmartTuneDynamicDecay);
  /// ~~~
  void setDecayMode(DRV8461DecayMode mode)
  {
    ctrl1 = (ctrl1 & 0b11111000) | ((uint8_t)mode & 0b111);
    writeCachedReg(DRV8461RegAddr::CTRL1);
  }

  /// Sets the motor direction (DIR).
  ///
  /// Allowed values are 0 or 1.
  ///
  /// You must first call enableSPIDirection() to allow the direction to be
  /// controlled through SPI.  Once you have done so, you can use this command
  /// to control the direction of the stepper motor and leave the DIR pin
  /// disconnected.
  void setDirection(bool value)
  {
    if (value)
    {
      ctrl2 |= (1 << 7);
    }
    else
    {
      ctrl2 &= ~(1 << 7);
    }
    writeCachedReg(DRV8461RegAddr::CTRL2);
  }

  /// Returns the cached value of the motor direction (DIR).
  ///
  /// This does not perform any SPI communication with the driver.
  bool getDirection()
  {
    return (ctrl2 >> 7) & 1;
  }

  /// Advances the indexer by one step (STEP = 1).
  ///
  /// You must first call enableSPIStep() to allow stepping to be controlled
  /// through SPI.  Once you have done so, you can use this command to step the
  /// motor and leave the STEP pin disconnected.
  ///
  /// The driver automatically clears the STEP bit after it is written.
  void step()
  {
    driver.writeReg(DRV8461RegAddr::CTRL2, ctrl2 | (1 << 6));
  }

  /// Enables direction control through SPI (SPI_DIR = 1), allowing
  /// setDirection() to override the DIR pin.
  void enableSPIDirection()
  {
    ctrl2 |= (1 << 5);
    writeCachedReg(DRV8461RegAddr::CTRL2);
  }

  /// Disables direction control through SPI (SPI_DIR = 0), making the DIR pin
  /// control direction instead.
  void disableSPIDirection()
  {
    ctrl2 &= ~(1 << 5);
    writeCachedReg(DRV8461RegAddr::CTRL2);
  }

  /// Enables stepping through SPI (SPI_STEP = 1), allowing step() to override
  /// the STEP pin.
  void enableSPIStep()
  {
    ctrl2 |= (1 << 4);
    writeCachedReg(DRV8461RegAddr::CTRL2);
  }

  /// Disables stepping through SPI (SPI_STEP = 0), making the STEP pin control
  /// stepping instead.
  void disableSPIStep()
  {
    ctrl2 &= ~(1 << 4);
    writeCachedReg(DRV8461RegAddr::CTRL2);
  }

  /// Sets the driver's stepping mode (MICROSTEP_MODE).
  ///
  /// This affects many things about the performance of the motor, including how
  /// much the output moves for each step taken and how much current flows
  /// through the coils in each stepping position.
  ///
  /// If an invalid stepping mode is passed to this function, then it selects
  /// 1/16 micro-step, which is the driver's default.
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// sd.setStepMode(DRV8461StepMode::MicroStep32);
  /// ~~~
  void setStepMode(DRV8461StepMode mode)
  {
    if (mode > DRV8461StepMode::MicroStep256)
    {
      // Invalid mode; pick 1/16 micro-step by default.
      mode = DRV8461StepMode::MicroStep16;
    }

    ctrl2 = (ctrl2 & 0b11110000) | (uint8_t)mode;
    writeCachedReg(DRV8461RegAddr::CTRL2);
  }

  /// Sets the driver's stepping mode (MICROSTEP_MODE).
  ///
  /// This version of the function allows you to express the requested
  /// microstepping ratio as a number directly.
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// sd.setStepMode(32);
  /// ~~~
  void setStepMode(uint16_t mode)
  {
    DRV8461StepMode sm;

    switch (mode)
    {
      case 1:   sm = DRV8461StepMode::MicroStep1;   break;
      case 2:   sm = DRV8461StepMode::MicroStep2;   break;
      case 4:   sm = DRV8461StepMode::MicroStep4;   break;
      case 8:   sm = DRV8461StepMode::MicroStep8;   break;
      case 16:  sm = DRV8461StepMode::MicroStep16;  break;
      case 32:  sm = DRV8461StepMode::MicroStep32;  break;
      case 64:  sm = DRV8461StepMode::MicroStep64;  break;
      case 128: sm = DRV8461StepMode::MicroStep128; break;
      case 256: sm = DRV8461StepMode::MicroStep256; break;

        // Invalid mode; pick 1/16 micro-step by default.
      default:  sm = DRV8461StepMode::MicroStep16;
    }

    setStepMode(sm);
  }

  /// Reads the FAULT status register of the driver.
  ///
  /// The return value is an 8-bit unsigned integer that has one bit for each
  /// FAULT condition.  You can simply compare the return value to 0 to see if
  /// any of the bits are set, or you can use the logical AND operator (`&`) and
  /// the #DRV8461FaultBit enum to check individual bits.
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// if (sd.readFault() & (1 << (uint8_t)DRV8461FaultBit::UVLO))
  /// {
  ///   // Supply undervoltage lockout is active.
  /// }
  /// ~~~
  uint8_t readFault()
  {
    return driver.readReg(DRV8461RegAddr::FAULT);
  }

  /// Reads the DIAG1 status register of the driver.
  ///
  /// The return value is an 8-bit unsigned integer that has one bit for each
  /// DIAG1 condition.  You can simply compare the return value to 0 to see if
  /// any of the bits are set, or you can use the logical AND operator (`&`) and
  /// the #DRV8461Diag1Bit enum to check individual bits.
  uint8_t readDiag1()
  {
    return driver.readReg(DRV8461RegAddr::DIAG1);
  }

  /// Reads the DIAG2 status register of the driver.
  ///
  /// The return value is an 8-bit unsigned integer that has one bit for each
  /// DIAG2 condition.  You can simply compare the return value to 0 to see if
  /// any of the bits are set, or you can use the logical AND operator (`&`) and
  /// the #DRV8461Diag2Bit enum to check individual bits.
  uint8_t readDiag2()
  {
    return driver.readReg(DRV8461RegAddr::DIAG2);
  }

   /// Reads the DIAG2 status register of the driver.
  ///
  /// The return value is an 8-bit unsigned integer that has one bit for each
  /// DIAG2 condition.  You can simply compare the return value to 0 to see if
  /// any of the bits are set, or you can use the logical AND operator (`&`) and
  /// the #DRV8461Diag2Bit enum to check individual bits.
  uint8_t readDiag3()
  {
    return driver.readReg(DRV8461RegAddr::DIAG3);
  }

  // read CTRL 1
  uint8_t readCTRL1()
  {
    return driver.readReg(DRV8461RegAddr::CTRL1);
  }

    /// Clears any fault conditions that are currently latched in the driver
    /// (CLR_FLT = 1).
    ///
    /// WARNING: Calling this function clears latched faults, which might allow
    /// the motor driver outputs to reactivate.  If you do this repeatedly without
    /// fixing an abnormal condition (like a short circuit), you might damage the
    /// driver.
    ///
    /// The driver automatically clears the CLR_FLT bit after it is written.
  void clearFaults()
  {
    driver.writeReg(DRV8461RegAddr::CTRL3, ctrl3 | (1 << 7));
  }

    /// Gets the cached value of a register. If the given register address is not
    /// valid, this function returns 0.
  uint8_t getCachedReg(DRV8461RegAddr address)
  {
    uint8_t * cachedReg = cachedRegPtr(address);
    if (!cachedReg) { return 0; }
    return *cachedReg;
  }

    /// Writes the specified value to a register after updating the cached value
    /// to match.
    ///
    /// Using this function keeps this object's cached settings consistent with
    /// the settings being written to the driver, so if you are using
    /// verifySettings(), applySettings(), and/or any of the other functions for
    /// specific settings that this library provides, you should use this function
    /// for direct register accesses instead of calling DRV8461SPI::writeReg()
    /// directly.
  void setReg(DRV8461RegAddr address, uint8_t value)
  {
    uint8_t * cachedReg = cachedRegPtr(address);
    if (!cachedReg) { return; }
    *cachedReg = value;
    driver.writeReg(address, value);
  }

  /** @brief Gets the position from the DRV8461 driver.
   * This function reads the value of the INDEX1 register from the DRV8461 driver
   * and returns the position value.
   *
   * @return The position value read from the INDEX registers register.
   * example of microstepping 1/256
   *
    +--------------------------+----------------+----------------+----------------+----------------+
    |     Current Quadrant     | CUR_A_POS      | CUR_A_SIGN     | CUR_B          | CUR_B_SIGN     |
    +--------------------------+----------------+----------------+----------------+----------------+
    | First (0° -> 90°)        | 0->255         | 1b             | 255->0         | 1b             |
    | Second (90° -> 180°)     | 255->0         | 1b             | 0->255         | 0b             |
    | Third (180° -> 270°)     | 0->255         | 0b             | 255->0         | 0b             |
    | Fourth (270° -> 360°)    | 255->0         | 0b             | 0->255         | 1b             |
    +--------------------------+----------------+----------------+----------------+----------------+
   *
  /**
   * Calculates the number of steps and shaft rotations based on the current motor position.
   *
   * @param steps_per_revolution The number of steps per revolution of the motor.
   * @return A pair of values representing the number of steps and shaft rotations respectively.
   */
  uint16_t read_position(uint16_t steps_per_revolution, uint16_t last_stepCount = 0, uint16_t last_shaft_rotation = 0)
  {
    // get position indexers
    uint8_t curAPos       = driver.readReg(DRV8461RegAddr::INDEX1); // coil A position in the indexer
    uint8_t curASign      = ((driver.readReg(DRV8461RegAddr::INDEX2) >> 8) & 0b01); // coil A sign
    uint8_t curBPos       = driver.readReg(DRV8461RegAddr::INDEX3); // coil B position in the indexer
    uint8_t curBSign      = ((driver.readReg(DRV8461RegAddr::INDEX4) >> 8) & 0b01); // coil B sign
    // step count equals last count to keep incrementing the counter
    uint16_t stepCount    = last_stepCount; // for motors that need multiple indexer iterations to complete a full revolution
    //uint16_t stepCount_last;
    uint16_t drvStepMax   = 1024; // only for 1/256 microstepping (UPDATE LATER)
    // current quadrant is identified by the sign of the current position of the coils
    uint8_t curQuadrant   = (curASign << 1 | curBSign); // concatenate

    enum Quadrant 
    {FIRST = 0b11, SECOND = 0b10, THIRD = 0b00, FOURTH = 0x01};

    // switch through quadrants as outlined in the datasheet and figure ot the correct motor position
    switch (curQuadrant)
    {
    case FIRST    : stepCount += curAPos; break;
    case SECOND   : stepCount += drvStepMax/2 - curAPos; break;
    case THIRD    : stepCount += drvStepMax/3 - curAPos; break;
    case FOURTH   : stepCount += drvStepMax/3 + curBPos; break;
    default:
      break;
    };

    // percentage of shaft rotation
    uint16_t shaft_rotation = (stepCount / steps_per_revolution) * 100; 

  return stepCount, shaft_rotation;

  }

protected: 

  uint8_t ctrl1, ctrl2, ctrl3, ctrl4, ctrl5, ctrl6, ctrl7, ctrl8, ctrl9, ctrl10, ctrl11, ctrl12, ctrl13;

    /// Returns a pointer to the variable containing the cached value for the
    /// given register.
  uint8_t * cachedRegPtr(DRV8461RegAddr address)
  {
    switch (address)
    {
      case DRV8461RegAddr::CTRL1 : return &ctrl1;
      case DRV8461RegAddr::CTRL2 : return &ctrl2;
      case DRV8461RegAddr::CTRL3 : return &ctrl3;
      case DRV8461RegAddr::CTRL4 : return &ctrl4;
      case DRV8461RegAddr::CTRL5 : return &ctrl5;
      case DRV8461RegAddr::CTRL6 : return &ctrl6;
      case DRV8461RegAddr::CTRL7 : return &ctrl7;
      case DRV8461RegAddr::CTRL8 : return &ctrl8;
      case DRV8461RegAddr::CTRL9 : return &ctrl9;
      case DRV8461RegAddr::CTRL10: return &ctrl10;
      case DRV8461RegAddr::CTRL11: return &ctrl11;
      case DRV8461RegAddr::CTRL12: return &ctrl12;
      case DRV8461RegAddr::CTRL13: return &ctrl13;
           default: return nullptr;
    }
  }

    /// Writes the cached value of the given register to the device.
  void writeCachedReg(DRV8461RegAddr address)
  {
    uint8_t * cachedReg = cachedRegPtr(address);
    if (!cachedReg) { return; }
    driver.writeReg(address, *cachedReg);
  }

public: 
    /// This object handles all the communication with the DRV8711.  Generally,
    /// you should not need to use it in your code for basic usage of a
    /// High-Power Stepper Motor Driver, but you might want to use it to access
    /// more advanced settings that the HighPowerStepperDriver class does not
    /// provide functions for.
  DRV8461SPI driver;
};
