all: receiver servoctl sync transmitter scandummy attitude-estimator

RECEIVER := ./cmd/receiver
SERVOCTL:= ./cmd/servoctl
SYNC := ./cmd/sync
TRANSMITTER := ./cmd/transmitter
SCAN_DUMMY := ./misc/scan-dummy
ATTITUDE_ESTIMATOR := ./attitude-estimator

receiver: $(RECEIVER)/receiver.go
	go build $(RECEIVER)/receiver.go

servoctl: $(SERVOCTL)/servoctl.go
	go build $(SERVOCTL)/servoctl.go

sync: $(SYNC)/sync.go
	go build $(SYNC)/sync.go $(SYNC)/servo.go $(SYNC)/accelerometer.go $(SYNC)/lidar.go $(SYNC)/data-buffer.go $(SYNC)/merger.go $(SYNC)/process.go 

transmitter: $(TRANSMITTER)/transmitter.go
	go build $(TRANSMITTER)/transmitter.go $(TRANSMITTER)/cloud.go

scandummy: $(SCAN_DUMMY)/scan-dummy.go
	go build  $(SCAN_DUMMY)/scan-dummy.go

attitude-estimator:  $(ATTITUDE_ESTIMATOR)/main.cpp
	g++ $(ATTITUDE_ESTIMATOR)/main.cpp $(ATTITUDE_ESTIMATOR)/attitude_estimator.cpp -o attitude-estimator

install:
	cp ./receiver /usr/local/bin
	cp ./servoctl /usr/local/bin
	cp ./sync /usr/local/bin
	cp ./transmitter /usr/local/bin

clean:
	rm -f receiver servoctl sync transmitter scan-dummy
