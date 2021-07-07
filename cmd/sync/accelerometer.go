package main

import (
	"bufio"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"log"
	"math"
	"time"

	"github.com/knei-knurow/frames"
)

// Supported data processing modes
const (
	AccelModeRaw = iota
	AccelModeDMP
)

// MPU-6050 constants. More details in the product documentation.
const (
	AccelScale2   = 16384.0
	AccelScale4   = 8192.0
	AccelScale8   = 4096.0
	AccelScale16  = 2048.0
	GyroScale250  = 131.0
	GyroScale500  = 65.5
	GyroScale1000 = 32.8
	GyroScale2000 = 16.4
)

// lidar-avr settings
const (
	AccelScaleDefault = AccelScale2
	GyroScaleDefault  = GyroScale250
	DeltaTimeDefault  = 0.02 // time in seconds between two measurements
)

// AccelData contains raw accel data (accel, gyro)
type AccelData struct {
	xAccel float64
	yAccel float64
	zAccel float64
	xGyro  float64
	yGyro  float64
	zGyro  float64
	timept time.Time
}

// AccelDataExt contains raw accel data (accel, gyro, mag)
type AccelDataExt struct {
	xAccel float64
	yAccel float64
	zAccel float64
	xGyro  float64
	yGyro  float64
	zGyro  float64
	xMag   float64
	yMag   float64
	zMag   float64
	timept time.Time
}

// AccelDataDMP contains accel data processed by Digital Motion Processor (quaternions)
type AccelDataDMP struct {
	qw     float64
	qx     float64
	qy     float64
	qz     float64
	timept time.Time
}

// AccelDataUnion is union-like structure which is used for sending accel data over channels
type AccelDataUnion struct {
	raw AccelData
	// rawExt AccelDataExt
	dmp AccelDataDMP
}

// Accel is the main accelerometer control struct
type Accel struct {
	mode        int
	calibration AccelData
	accelScale  float64
	gyroScale   float64
	deltaTime   float64
	port        io.Reader
	data        AccelDataUnion
	process     Process
}

// MPU-6050 predefined calibrations
var (
	accelCalib = AccelData{
		// POSSIBLE ERROR SOURCE: Values differ depending on the temperature
		xAccel: 812.0,
		yAccel: 118.0,
		zAccel: -14750.0 + AccelScale2,
		xGyro:  55.0,
		yGyro:  -56.0,
		zGyro:  39.0,
	}
	noAccelCalib = AccelData{
		xAccel: 0,
		yAccel: 0,
		zAccel: 0,
		xGyro:  0,
		yGyro:  0,
		zGyro:  0,
	}
)

// StartLoop starts the accelerometer main loop
func (accel *Accel) StartLoop(channel chan AccelDataUnion) (err error) {
	if err := accel.process.StartProcess(); err != nil {
		return fmt.Errorf("start accel process: %v", err)
	}

	scanner := bufio.NewScanner(accel.process.Stdout)
	scanner.Split(bufio.ScanLines)

	for {
		accel.ReadData()
		accel.PreprocessData()

		for scanner.Scan() {

		}
		if err := scanner.Err(); err != nil {
			return err
		}

		channel <- accel.data
	}
}

// ProcessAccelFrame takes data frame containing raw accelerometer measurements and tries
// to unpack them and store in the accel struct.
func (accel *Accel) ProcessAccelFrame(frame frames.Frame) (err error) {
	timept := time.Now()

	if frame[0] != 'L' ||
		frame[1] != 'D' ||
		frame[2] != 12 ||
		frame[3] != '+' {
		return errors.New("bad frame begin")
	}

	if !frames.Verify(frame) {
		return errors.New("bad checksum")
	}

	fdata := frame.Data()
	accel.data.raw.timept = timept // POSSIBLE ERROR SOURCE: Time of data receipt
	accel.data.raw.xAccel = float64(mergeBytes(fdata[0], fdata[1]))
	accel.data.raw.yAccel = float64(mergeBytes(fdata[2], fdata[3]))
	accel.data.raw.zAccel = float64(mergeBytes(fdata[4], fdata[5]))
	accel.data.raw.xGyro = float64(mergeBytes(fdata[6], fdata[7]))
	accel.data.raw.yGyro = float64(mergeBytes(fdata[8], fdata[9]))
	accel.data.raw.zGyro = float64(mergeBytes(fdata[10], fdata[11]))

	return nil
}

// ProcessAccelFrameDMP takes data frame containing DMP-processed accelerometer measurements
// and tries to unpack them and store in the accel struct.
func (accel *Accel) ProcessAccelFrameDMP(frame frames.Frame) (err error) {
	timept := time.Now()

	if frame[0] != 'L' ||
		frame[1] != 'Q' ||
		frame[2] != 16 ||
		frame[3] != '+' {
		return errors.New("bad frame begin")
	}

	if !frames.Verify(frame) {
		return errors.New("bad checksum")
	}

	fdata := frame.Data()
	accel.data.dmp.timept = timept // POSSIBLE ERROR SOURCE: Time of data receipt
	accel.data.dmp.qw = float32frombytes(fdata[0:4])
	accel.data.dmp.qx = float32frombytes(fdata[4:8])
	accel.data.dmp.qy = float32frombytes(fdata[8:12])
	accel.data.dmp.qz = float32frombytes(fdata[12:16])

	return nil
}

// PreprocessData converts raw accel data to X * gravitational_acceleration and gyro to deg/s
func (accel *Accel) PreprocessData() {
	accel.data.raw.xAccel = (accel.data.raw.xAccel + accel.calibration.xAccel) / accel.accelScale
	accel.data.raw.yAccel = (accel.data.raw.yAccel + accel.calibration.yAccel) / accel.accelScale
	accel.data.raw.zAccel = (accel.data.raw.zAccel + accel.calibration.zAccel) / accel.accelScale
	accel.data.raw.xGyro = (accel.data.raw.xGyro + accel.calibration.xGyro) / accel.gyroScale
	accel.data.raw.yGyro = (accel.data.raw.yGyro + accel.calibration.yGyro) / accel.gyroScale
	accel.data.raw.zGyro = (accel.data.raw.zGyro + accel.calibration.zGyro) / accel.gyroScale
}

// mergeBytes merges two bytest to int
func mergeBytes(left8 byte, right8 byte) int {
	v := int((uint16(left8) << 8) | uint16(right8))
	// awesome conversion to signed int
	if v >= 32768 {
		v = -65536 + v
	}
	return v
}

// float32frombytes converts 4 bytes to float32
func float32frombytes(bytes []byte) float64 {
	bits := binary.LittleEndian.Uint32(bytes)
	float := math.Float32frombits(bits)
	return float64(float)
}

// ReadData reads and parses new measurement
func (accel *Accel) ReadData() (err error) {
	var frame frames.Frame
	var frameLen int
	if accel.mode == AccelModeRaw {
		frameLen = 18
	} else {
		frameLen = 22
	}

	frame = make(frames.Frame, frameLen)
	if err := accel.ReadAccelFrame(frame, frameLen); err != nil {
		log.Printf("error: %s\n", err)
	}

	if accel.mode == AccelModeRaw {
		err = accel.ProcessAccelFrame(frame)
	} else {
		err = accel.ProcessAccelFrameDMP(frame)
	}
	if err != nil {
		return errors.New("cannot process frame")
	}
	return nil
}

// ReadAccelFrame is a low level function to read an accelerometer frame
func (accel *Accel) ReadAccelFrame(data []byte, length int) (err error) {
	scan := false
	for i := 0; i < length; i++ {
		buf := make([]byte, 1)
		_, err := accel.port.Read(buf)
		if err != nil {
			return errors.New("cannot read from port")
		}

		if scan {
			data[i] = buf[0]
		} else {
			if buf[0] == 'L' {
				scan = true
				data[i] = buf[0]
			} else {
				return errors.New("lost some data")
			}
		}
	}
	return nil
}
