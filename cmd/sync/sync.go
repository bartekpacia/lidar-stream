package main

import (
	"bufio"
	"flag"
	"fmt"
	"log"
	"os"
	"time"

	"github.com/tarm/serial"
)

var (
	avrPortName   string
	avrBaudRate   int
	lidarPortName string
	lidarMode     int
	lidarRPM      int
	lidarExe      string
	accelOut      bool
	servoOut      bool
	lidarOut      bool
)

func init() {
	log.SetFlags(0)
	log.SetPrefix("sync: ")

	flag.StringVar(&avrPortName, "avrport", "COM13", "AVR serial communication port")
	flag.IntVar(&avrBaudRate, "avrbaud", 19200, "port baud rate (bps)")

	flag.StringVar(&lidarExe, "lidarexe", "lidar.exe", "lidar-scan executable")
	flag.StringVar(&lidarPortName, "lidarport", "COM4", "RPLIDAR serial communication port")
	flag.IntVar(&lidarMode, "lidarmode", rplidarModeDefault, "RPLIDAR mode")
	flag.IntVar(&lidarRPM, "lidarpm", 660, "RPLIDAR given revolutions per minute")

	flag.BoolVar(&accelOut, "accel", false, "print accelerometer data on stdout")
	flag.BoolVar(&servoOut, "servo", false, "print set servo position on stdout")
	flag.BoolVar(&lidarOut, "lidar", false, "print lidar data on stdout")

	flag.Parse()
	log.Println("starting...")
}

func main() {
	writer := bufio.NewWriter(os.Stdout)

	config := &serial.Config{
		Name: avrPortName,
		Baud: avrBaudRate,
	}
	port, err := serial.OpenPort(config)
	if err != nil {
		log.Println("cannot open port:", err)
		return
	}
	defer port.Close()

	// Sources of data initialization
	accel := Accel{
		calibration: accelCalib,
		accelScale:  AccelScaleDefault,
		gyroScale:   GyroScaleDefault,
		deltaTime:   DeltaTimeDefault,
		port:        port,
		mode:        AccelModeRaw,
	}

	servo := Servo{
		data:       ServoData{positon: servoStartPos},
		positonMin: servoMinPos,
		positonMax: servoMaxPos,
		vector:     100,
		port:       port,
		delayMs:    100,
	}
	log.Println("setting the servo to the start position")
	servo.SetPosition(servoStartPos)
	log.Println("waiting for the servo")
	time.Sleep(time.Second) // to be sure that the servo is on the right position

	// lidar := Lidar{ // TODO: make it more configurable from command line
	// 	RPM:  lidarRPM,
	// 	Mode: lidarMode,
	// 	Args: []string{lidarPortName, "--rpm", fmt.Sprint(lidarRPM), "--mode", fmt.Sprint(lidarMode)},
	// 	Path: lidarExe, // TODO: Check if exists
	// }

	// Create communication channels
	// lidarChan := make(chan *LidarCloud) // LidarCloud is >64kB so it cannot be directly passed by a channel
	servoChan := make(chan ServoData)
	accelChan := make(chan AccelDataUnion)

	// Create data buffers
	accelBuffer := NewAccelDataBuffer(32)
	servoBuffer := NewServoDataBuffer(32)
	// var lidarBuffer *LidarCloud

	// Start goroutines
	// go lidar.StartLoop(lidarChan)
	go servo.StartLoop(servoChan)
	go accel.StartLoop(accelChan)

	// Main loop
	for {
		select {
		// case lidarData := <-lidarChan:
		// 	if lidarOut {
		// 		writer.WriteString(fmt.Sprintf("L %d %d\n", lidarData.ID, lidarData.Size))
		// 	}
		// lidarBuffer = lidarData
		case servoData := <-servoChan:
			if servoOut {
				writer.WriteString(fmt.Sprintf("S %d %d\n", servoData.timept.UnixNano(), servoData.positon))
			}
			servoBuffer.Append(servoData)
		case accelData := <-accelChan:
			if accelOut {
				if accel.mode == AccelModeRaw {
					// writer.WriteString(fmt.Sprintf("A %d\t%d\t%d\t%d\t%d\t%d\t%d\n",
					// 	accelData.raw.timept.UnixNano(),
					// 	accelData.raw.xAccel, accelData.raw.yAccel, accelData.raw.zAccel,
					// 	accelData.raw.xGyro, accelData.raw.yGyro, accelData.raw.zGyro))
					writer.WriteString(fmt.Sprintf("%f\t%f\t%f\t%f\t%f\t%f\n",
						accelData.raw.xAccel, accelData.raw.yAccel, accelData.raw.zAccel,
						accelData.raw.xGyro, accelData.raw.yGyro, accelData.raw.zGyro))
				} else {
					writer.WriteString(fmt.Sprintf("a %d\t%f\t%f\t%f\t%f\n",
						accelData.dmp.timept.UnixNano(),
						accelData.dmp.qw, accelData.dmp.qx, accelData.dmp.qy, accelData.dmp.qz))
				}
			}
			accelBuffer.Append(accelData)
		}
		writer.Flush()

		// go mergerLidarServoV1(lidarBuffer, &servoBuffer, true)
	}
}
