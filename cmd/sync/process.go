package main

import (
	"fmt"
	"io"
	"log"
	"os"
	"os/exec"
	"runtime"
)

type Process struct {
	Args    []string       // Argv.
	Path    string         // Path to executable.
	Stdout  io.ReadCloser  // Stdout.
	Stderr  io.ReadCloser  // Stderr.
	Stdin   io.WriteCloser // Stderr.
	process *exec.Cmd      // Process object.
}

// StartProcess starts the process.
// It does not check whether it has been already started.
func (process *Process) StartProcess() error {
	process.process = exec.Command(process.Path, process.Args...)
	log.Println("starting", process.Path, "process with args:", process.Args)

	// stdout
	var err error
	process.Stdout, err = process.process.StdoutPipe()
	if err != nil {
		return fmt.Errorf("get stdout of process: %v", err)
	}

	// stdin
	process.Stdin, err = process.process.StdinPipe()
	if err != nil {
		return fmt.Errorf("get stdin of process: %v", err)
	}

	// stderr
	process.process.Stderr = os.Stderr
	process.Stderr = os.Stderr

	err = process.process.Start()
	if err != nil {
		return fmt.Errorf("start process: %v", err)
	}

	return nil
}

// CloseProcess sends SIGINT (ctrl+c) to the process.
// It is important because process may perform cleanup on SIGINT.
// It does not check whether it has been already started.
//
// Does not work on Windows - instead, it just calls KillProcess.
func (process *Process) CloseProcess() (err error) {
	if runtime.GOOS == "windows" {
		log.Println("closing is not implemented on Windows, killing instead")
		return process.KillProcess()
	}

	log.Println("closing lidar-scan process")
	if err = process.process.Process.Signal(os.Interrupt); err != nil {
		return fmt.Errorf("send SIGINT to the process: %v", err)
	}

	return nil
}

// KillProcess kills the process immediately, so the cleanup will not be performed.
//
// Use it only in emergency situations. Prefer CloseProcess.
func (process *Process) KillProcess() (err error) {
	log.Println("killing the process")
	if err = process.process.Process.Kill(); err != nil {
		return fmt.Errorf("kill the process: %v", err)
	}
	return nil
}

// UpdateProcessArgs modifies the parametres passed to the process to change its behavior.
//
// In practice, there is no way to modify the command-line arguments passed to the process
// while it is running, so this function simply kills the process and starts it again
// with updated args.
// Such a solution should be sufficient for most cases.
func (process *Process) UpdateProcessArgs(args []string) (err error) {
	log.Println("changing process args")
	if err := process.CloseProcess(); err != nil {
		return err
	}

	process.Args = args

	if err := process.StartProcess(); err != nil {
		return err
	}
	return nil
}
