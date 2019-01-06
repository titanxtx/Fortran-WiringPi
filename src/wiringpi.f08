!MIT License

!Copyright (c) 2019 Joshua Oliva

!Permission is hereby granted, free of charge, to any person obtaining a copy
!of this software and associated documentation files (the "Software"), to deal
!in the Software without restriction, including without limitation the rights
!to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
!copies of the Software, and to permit persons to whom the Software is
!furnished to do so, subject to the following conditions:

!The above copyright notice and this permission notice shall be included in all
!copies or substantial portions of the Software.

!THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
!IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
!FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
!AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
!LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
!OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
!SOFTWARE.
module wiringpi
use iso_c_binding,only:c_int
implicit none

integer(kind=c_int),parameter::WPI_MODE_PINS=0,&
WPI_MODE_GPIO=1,&
WPI_MODE_GPIO_SYS=2,&
WPI_MODE_PHYS=3,&
WPI_MODE_PIFACE=4,&
WPI_MODE_UNINITIALISED=-1,&
INPUT=0,&
OUTPUT=1,&
PWM_OUTPUT=2,&
GPIO_CLOCK=3,&
SOFT_PWM_OUTPUT=4,&
SOFT_TONE_OUTPUT=5,&
PWM_TONE_OUTPUT=6,&
LOW=0,&
HIGH=1,&

!Pull up/down/none

PUD_OFF=0,&
PUD_DOWN=1,&
PUD_UP=2,&

!PWM

PWM_MODE_MS=0,&
PWM_MODE_BAL=1,&

! Interrupt levels

INT_EDGE_SETUP=0,&
INT_EDGE_FALLING=1,&
INT_EDGE_RISING=2,&
INT_EDGE_BOTH=3,&
PI_MODEL_A=0,&
PI_MODEL_B=1,&
PI_MODEL_AP=2,&
PI_MODEL_BP=3,&
PI_MODEL_2=4,&
PI_ALPHA=5,&
PI_MODEL_CM=6,&
PI_MODEL_07=7,&
PI_MODEL_3=8,&
PI_MODEL_ZERO=9,&
PI_MODEL_CM3=10,&
PI_MODEL_ZERO_W=12,&
PI_MODEL_3P=13,&

PI_VERSION_1=0,&
PI_VERSION_1_1=1,&
PI_VERSION_1_2=2,&
PI_VERSION_2=3,&

PI_MAKER_SONY=0,&
PI_MAKER_EGOMAN=1,&
PI_MAKER_EMBEST=2,&
PI_MAKER_UNKNOWN=3

    interface 
        integer(kind=c_int) function wiringPiSetup() bind(C,name="wiringPiSetup")
            use iso_c_binding,only:c_int
        end function wiringPiSetup

        integer(kind=c_int) function wiringPiSetupGpio() bind(C,name="wiringPiSetupGpio")
            use iso_c_binding,only:c_int
        end function wiringPiSetupGpio

        integer(kind=c_int) function wiringPiSetupSys() bind(C,name="wiringPiSetupSys")
            use iso_c_binding,only:c_int
        end function wiringPiSetupSys

        integer(kind=c_int) function wiringPiSetupPhys() bind(C,name="wiringPiSetupPhys")
            use iso_c_binding,only:c_int
        end function wiringPiSetupPhys

        subroutine pinMode(pin,mode) bind(C,name="pinMode")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pin,mode
        end subroutine pinMode

        subroutine pinModeAlt(pin,mode) bind(C,name="pinModeAlt")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pin,mode
        end subroutine pinModeAlt

        subroutine analogWrite(pin,value) bind(C,name="analogWrite")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pin,value
        end subroutine analogWrite

        subroutine digitalWrite(pin,value) bind(C,name="digitalWrite")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pin,value
        end subroutine digitalWrite

        subroutine digitalWriteByte(value) bind(C,name="digitalWriteByte")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::value
        end subroutine digitalWriteByte

        subroutine digitalWriteByte2(value) bind(C,name="digitalWriteByte2")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::value
        end subroutine digitalWriteByte2

        subroutine pwmWrite(pin,value) bind(C,name="pwmWrite")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pin,value
        end subroutine pwmWrite

        integer(kind=c_int) function digitalRead(pin) bind(C,name="digitalRead")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pin
        end function digitalRead

        integer(kind=c_int) function digitalRead8(pin) bind(C,name="digitalRead8")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pin
        end function digitalRead8

        integer(kind=c_int) function analogRead(pin) bind(C,name="analogRead")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pin
        end function analogRead

        subroutine pullUpDnControl(pin,pud) bind(C,name="pullUpDnControl")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pin,pud
        end subroutine pullUpDnControl

        subroutine pwmSetMode(mode) bind(C,name="pwmSetMode")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::mode
        end subroutine pwmSetMode

        subroutine pwmSetRange(range) bind(C,name="pwmSetRange")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::range
        end subroutine pwmSetRange

        subroutine pwmSetClock(divisor) bind(C,name="pwmSetClock")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::divisor
        end subroutine pwmSetClock

        integer(kind=c_int) function millis() bind(C,name="millis")
            use iso_c_binding,only:c_int
        end function millis

        integer(kind=c_int) function micros() bind(C,name="millis")
            use iso_c_binding,only:c_int
        end function micros

        subroutine delay(howLong) bind(C,name="delay")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::howLong
        end subroutine delay

        subroutine delayMicroseconds(howLong) bind(C,name="delayMicroseconds")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::howLong
        end subroutine delayMicroseconds
        
        integer(kind=c_int) function piHiPri(priority) bind(C,name="piHiPri")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::priority
        end function piHiPri

        integer(kind=c_int) function waitForInterrupt(pin,timeOut) bind(C,name="waitForInterrupt")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pin,timeOut
        end function waitForInterrupt

        integer(kind=c_int) function wiringPiISR(pin,edgeType,func) bind(C,name="wiringPiISR")
            use iso_c_binding,only:c_int,c_ptr
            integer(kind=c_int),intent(in),value::pin,edgeType
            type(c_ptr),intent(in),value::func
        end function wiringPiISR

        integer(kind=c_int) function piThreadCreate(func) bind(C,name="piThreadCreate")
            use iso_c_binding,only:c_int,c_ptr
            type(c_ptr),intent(in),value::func
        end function piThreadCreate

        subroutine piLock(key) bind(C,name="piLock")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::key
        end subroutine piLock

        subroutine piUnlock(key) bind(C,name="piUnlock")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::key
        end subroutine piUnlock

        integer(kind=c_int) function piBoardRev() bind(C,name="piBoardRev")
            use iso_c_binding,only:c_int
        end function piBoardRev

        integer(kind=c_int) function wpiPinToGpio(wpiPin) bind(C,name="wpiPinToGpio")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::wpiPin
        end function wpiPinToGpio

        integer(kind=c_int) function physPinToGpio(physPin) bind(C,name="physPinToGpio")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::physPin
        end function physPinToGpio

        integer(kind=c_int) function setPadDrive(group,value) bind(C,name="setPadDrive")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::group,value
        end function setPadDrive

        subroutine piBoardId(model,rev,mem,maker,overVolted) bind(C,name="piBoardId")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in)::model,rev,mem,maker,overVolted
        end subroutine piBoardId

        integer(kind=c_int) function piGpioLayout() bind(C,name="piGpioLayout")
            use iso_c_binding,only:c_int
        end function piGpioLayout

        integer(kind=c_int) function getAlt(pin) bind(C,name="getAlt")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pin
        end function getAlt

        subroutine pwmToneWrite(pin,freq) bind(C,name="pwmToneWrite")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pin,freq
        end subroutine pwmToneWrite

        subroutine gpioClockSet(pin,freq) bind(C,name="gpioClockSet")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pin,freq
        end subroutine gpioClockSet

        integer(kind=c_int) function digitalReadByte() bind(C,name="digitalReadByte")
            use iso_c_binding,only:c_int
        end function digitalReadByte

        integer(kind=c_int) function digitalReadByte2(pin) bind(C,name="digitalReadByte2")
            use iso_c_binding,only:c_int
        end function digitalReadByte2


        !--------Serial_library--------------------
        integer(kind=c_int) function serialOpen(device,baud) bind(C,name="serialOpen")
            use iso_c_binding,only:c_int,c_ptr
            integer(kind=c_int),intent(in),value::baud
            type(c_ptr),intent(in),value::device
        end function serialOpen

        subroutine serialClose(fd) bind(C,name="serialClose")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::fd
        end subroutine serialClose

        subroutine serialPutchar(fd,c) bind(C,name="serialPutchar")
            use iso_c_binding,only:c_int,c_ptr
            integer(kind=c_int),intent(in),value::fd
            type(c_ptr),intent(in),value::c
        end subroutine serialPutchar

        subroutine serialPuts(fd,s) bind(C,name="serialPuts")
            use iso_c_binding,only:c_int,c_ptr
            integer(kind=c_int),intent(in),value::fd
            type(c_ptr),intent(in),value::s
        end subroutine serialPuts

        subroutine serialPrintf(fd,message) bind(C,name="serialPrintf")
            use iso_c_binding,only:c_int,c_ptr
            integer(kind=c_int),intent(in),value::fd
            type(c_ptr),intent(in),value::message
        end subroutine serialPrintf

        integer(kind=c_int) function serialDataAvail(fd) bind(C,name="serialOpen")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::fd
        end function serialDataAvail

        integer(kind=c_int) function serialGetchar(fd) bind(C,name="serialOpen")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::fd
        end function serialGetchar

        subroutine serialFlush(fd) bind(C,name="piUnlock")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::fd
        end subroutine serialFlush

        !-----SPI library
        integer(kind=c_int) function wiringPiSPISetup(channel,speed) bind(C,name="wiringPiSPISetup")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::channel,speed
        end function wiringPiSPISetup

        integer(kind=c_int) function wiringPiSPIDataRW(channel,data,len_) bind(C,name="wiringPiSPIDataRW")
            use iso_c_binding,only:c_int,c_ptr
            integer(kind=c_int),intent(in),value::channel,len_
            type(c_ptr),intent(in),value::data
        end function wiringPiSPIDataRW

        !----I2c library

        integer(kind=c_int) function wiringPiI2CSetup(devId) bind(C,name="wiringPiI2CSetup")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::devId
        end function wiringPiI2CSetup

        integer(kind=c_int) function wiringPiI2CRead(fd) bind(C,name="wiringPiI2CRead")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::fd
        end function wiringPiI2CRead

        integer(kind=c_int) function wiringPiI2CWrite(fd,data) bind(C,name="wiringPiI2CWrite")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::fd,data
        end function wiringPiI2CWrite

        integer(kind=c_int) function wiringPiI2CWriteReg8(fd,reg,data) bind(C,name="wiringPiI2CWriteReg8")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::fd,reg,data
        end function wiringPiI2CWriteReg8

        integer(kind=c_int) function wiringPiI2CWriteReg16(fd,reg,data) bind(C,name="wiringPiI2CWriteReg16")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::fd,reg,data
        end function wiringPiI2CWriteReg16

        integer(kind=c_int) function wiringPiI2CReadReg8(fd,reg) bind(C,name="wiringPiI2CReadReg8")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::fd,reg
        end function wiringPiI2CReadReg8

        integer(kind=c_int) function wiringPiI2CReadReg16(fd,reg) bind(C,name="wiringPiI2CReadReg16")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::fd,reg
        end function wiringPiI2CReadReg16

        !--------Shift library-----------------------
        integer(kind=c_short) function shiftIn(dPin,cPin,order) bind(C,name="shiftIn")
            use iso_c_binding,only:c_short
            integer(kind=c_short),intent(in),value::dPin,cPin,order
        end function shiftIn

        subroutine shiftOut(dPin,cPin,order,val) bind(C,name="shiftOut")
            use iso_c_binding,only:c_short
            integer(kind=c_short),intent(in),value::dPin,cPin,order,val
        end subroutine shiftOut
        !-------Software PWM library-----------------

        integer(kind=c_int) function softPwmCreate(pin,initialValue,pwmRange) bind(C,name="softPwmCreate")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pin,initialValue,pwmRange
        end function softPwmCreate

        subroutine softPwmWrite(pin,value) bind(C,name="softPwmWrite")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pin,value
        end subroutine softPwmWrite

        !-------Software Tone library-----------------

        integer(kind=c_int) function softToneCreate(pin) bind(C,name="softToneCreate")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pin
        end function softToneCreate

        subroutine softToneWrite(pin,freq) bind(C,name="softToneWrite")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pin,freq
        end subroutine softToneWrite

        !------Extensions------------------------------
        integer(kind=c_int) function mcp23017Setup(pinBase,i2cAddress) bind(C,name="mcp23017Setup")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pinBase,i2cAddress
        end function mcp23017Setup

        integer(kind=c_int) function mcp23008Setup(pinBase,i2cAddress) bind(C,name="mcp23008Setup")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pinBase,i2cAddress
        end function mcp23008Setup

        integer(kind=c_int) function mcp23s08Setup(pinBase,spiPort,devId) bind(C,name="mcp23s08Setup")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pinBase,spiPort,devId
        end function mcp23s08Setup

        integer(kind=c_int) function sr595Setup(pinBase,numPins,dataPin,clockPin,latchPin) bind(C,name="sr595Setup")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pinBase,numPins,dataPin,clockPin,latchPin
        end function sr595Setup

        integer(kind=c_int) function pcf8574Setup(pinBase,i2cAddress) bind(C,name="pcf8574Setup")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pinBase,i2cAddress
        end function pcf8574Setup

        integer(kind=c_int) function pcf8591Setup(pinBase,i2cAddress) bind(C,name="pcf8591Setup")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pinBase,i2cAddress
        end function pcf8591Setup

        integer(kind=c_int) function sn3218Setup(pinBase) bind(C,name="sn3218Setup")
            use iso_c_binding,only:c_int
            integer(kind=c_int),intent(in),value::pinBase
        end function sn3218Setup
    end interface

end module wiringpi
