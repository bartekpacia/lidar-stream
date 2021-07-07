package main

import (
	"fmt"
	"io"
	"log"
	"time"

	"github.com/knei-knurow/frames"
)

// Servo constants
const (
	servoMinPos    = 1000
	servoStartPos  = 2500
	servoMaxPos    = 3000
	servoUnitToDeg = 0.1 // 1 servo position unit = servoUnitToDeg * deg
)

// ServoData is a struct containing information about servo state
type ServoData struct {
	positon uint16    // given, not real
	timept  time.Time // time of sending a new position to the servo
}

// Servo is the main servo control struct
type Servo struct {
	data       ServoData // servo data
	positonMax uint16    // max position
	positonMin uint16    // min position
	vector     uint16    //
	port       io.Writer // port to write controlling frames
	delayMs    uint      // ms delay between orders
}

// Move sends the move order to the servo and updates its movement vector.
func (servo *Servo) Move() {
	servo.data.positon += servo.vector

	switch {
	case servo.data.positon < servo.positonMin:
		servo.data.positon = servo.positonMin
		servo.vector = -servo.vector
	case servo.data.positon > servo.positonMax:
		servo.data.positon = servo.positonMax
		servo.vector = -servo.vector
	}
}

// SendData is a low-level function to create a data frame and send it via serial port
func (servo *Servo) SendData() (err error) {
	inputByte := servo.data.positon
	data := []byte{byte(inputByte >> 8), byte(inputByte)}
	f := frames.Create([2]byte{'L', 'D'}, data)
	for i, currentByte := range f {
		if _, err := servo.port.Write([]byte{currentByte}); err != nil {
			return fmt.Errorf("cannot write data (byte %d) to port: %s", i, err)
		}
	}

	// POSSIBLE SOURCE OF ERRORS: that's the frame send time, not the actual servo set time
	servo.data.timept = time.Now()
	return nil
}

// SetPosition sends an order with new position to the servo. The value is not checked
// by this function but might be checked by AVR software.
func (servo *Servo) SetPosition(pos uint16) (err error) {
	servo.data.positon = pos
	return servo.SendData()
}

// StartLoop starts a loop responsible for controlling the servo position
// and updading the channel with its new calculated position.
func (servo *Servo) StartLoop(channel chan ServoData) {
	for {
		servo.Move()
		if err := servo.SendData(); err != nil {
			log.Println("unable to send servo data:", err)
		}
		channel <- servo.data

		if servo.delayMs != 0 {
			time.Sleep(time.Millisecond * time.Duration(servo.delayMs))
		}
	}
}
