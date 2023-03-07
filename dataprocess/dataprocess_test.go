package dataprocess

import (
	"os"
	"testing"

	"github.com/edaniels/golog"
	pc "go.viam.com/rdk/pointcloud"
	"go.viam.com/test"
)

func TestWriteBytesToFile(t *testing.T) {
	_ = golog.NewTestLogger(t)
	tempDir, err := os.MkdirTemp("", "*")
	defer os.RemoveAll(tempDir)
	test.That(t, err, test.ShouldBeNil)
	// Save a set of bytes to a file
	actualBytes := []byte{1, 5, 8}
	fileDest := tempDir + "test_bytes"
	err = WriteBytesToFile(actualBytes, fileDest)
	test.That(t, err, test.ShouldBeNil)
	// Test that the file was actually written
	readBytes, err := os.ReadFile(fileDest)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, readBytes, test.ShouldResemble, actualBytes)
}

func TestWritePCDToFile(t *testing.T) {
	_ = golog.NewTestLogger(t)
	tempDir, err := os.MkdirTemp("", "*")
	defer os.RemoveAll(tempDir)
	test.That(t, err, test.ShouldBeNil)
	fileDest := tempDir + "test_pcd.pcd"
	pointcloud := pc.New()
	err = WritePCDToFile(pointcloud, fileDest)
	test.That(t, err, test.ShouldBeNil)
	// Test that the file was actually written
	_, err = os.Stat(fileDest)
	test.That(t, err, test.ShouldBeNil)
}
