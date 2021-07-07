package main

import (
	"bufio"
	"errors"
	"fmt"
	"log"
	"time"
)

// Lidar-related constants.
const (
	lidarMaxDataSize = 8192 // defined by RPLIDAR hardware

	// Lidar scanning modes.
	rplidarModeBoost       = 2
	rplidarModeSensitivity = 3 // best for indoor applications
	rplidarModeStability   = 4 // best for very sunny days (like "Dni Knurowa 2021")
	rplidarModeDefault     = rplidarModeSensitivity
)

// Point is a single angle + distance measurement.
type Point struct {
	Angle float32 // Angle in degrees.
	Dist  float32 // Distance in millimeters.
}

// LidarCloud is one full (360deg) lidar point cloud.
type LidarCloud struct {
	ID        int                     //
	TimeBegin time.Time               // time point of starting line read (line starting with '!').
	TimeDiff  int                     // number of milliseconds of current cloud measurement (received from lidar-scan).
	timeEnd   time.Time               // timeBegin increased by timeDiff milliseconds
	Data      [lidarMaxDataSize]Point // Measurements data.
	Size      uint                    // Number of used points in Data.
	Ready     bool
}

// Lidar represents general lidar parameters.
type Lidar struct {
	TimeInit           time.Time // Time of the first starting line read (line starting with '!').
	RPM                int       // Declared RPM (actual may differ).
	Mode               int       // rplidar scan mode.
	Process            Process   // lidar-scan process.
	running            bool      // Whether lidar-scan is currently scanning.
	nextCloudCount     int
	nextCloudTimeDiff  int
	nextCloudTimeBegin time.Time
}

// StartLoop starts the lidar-scan process and runs a loop responsible for reading and
// processing lidar data from redirected lidar-scan's stdout. It is designed to be run in a
// goroutine. The channel sends pointers to LidarCloud which contains the latest scanned
// point cloud. The pointers approach is required because LidarCloud is greather than 64kB
// which is a Go limit.
func (lidar *Lidar) StartLoop(channel chan *LidarCloud) (err error) {
	if err := lidar.Process.StartProcess(); err != nil {
		return fmt.Errorf("start process: %v", err)
	}

	scanner := bufio.NewScanner(lidar.Process.Stdout)
	scanner.Split(bufio.ScanLines)
	for {
		// create new cloud every time to pass the pointer via channel and avoid data race
		cloud := LidarCloud{
			ID:        lidar.nextCloudCount + 1,
			TimeDiff:  lidar.nextCloudTimeDiff,
			TimeBegin: lidar.nextCloudTimeBegin, // POSSIBLE ERROR SOURCE: using milliseconds by lidar-scan
			timeEnd:   lidar.nextCloudTimeBegin.Add(time.Millisecond * time.Duration(lidar.nextCloudTimeDiff))}

		for scanner.Scan() {
			line := scanner.Text()

			if err := lidar.ProcessLine(line, &cloud); err != nil {
				log.Printf("unable to parse line: %s\n", err)
				// TODO: buffer overflow error handling (but tbh it never happens)
			}

			if cloud.Ready {
				channel <- &cloud
				break // in order to create new LidarCloud
			}
		}
		if err := scanner.Err(); err != nil {
			return err
		}
	}
}

// ProcessLine takes a single line from lidar-scan stdout, processes it, and modifies cloud.
func (lidar *Lidar) ProcessLine(line string, cloud *LidarCloud) (err error) {
	if len(line) == 0 {
		return
	}

	switch line[0] {
	case '#':
	case '!':
		lidar.nextCloudTimeBegin = time.Now() // POSSIBLE ERROR SOURCE: using time of data receive
		if _, err := fmt.Sscanf(line, "! %d %d", &lidar.nextCloudCount, &lidar.nextCloudTimeDiff); err != nil {
			return errors.New("invalid starting line")
		}
		cloud.Ready = true
	default:
		var angle, dist float32
		if _, err := fmt.Sscanf(line, "%f %f", &angle, &dist); err != nil {
			return fmt.Errorf("invalid data line: \"%s\"", line)
		}

		cloud.Size++
		if cloud.Size >= lidarMaxDataSize {
			return errors.New("data buffer overflow")
		}
		cloud.Data[cloud.Size] = Point{angle, dist}
	}
	return nil
}
