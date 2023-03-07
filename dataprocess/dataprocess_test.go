package dataprocess

import (
	"os"
	"testing"
	"time"

	"github.com/edaniels/golog"
	pc "go.viam.com/rdk/pointcloud"
	"go.viam.com/test"
)

func TestWriteBytesToFile(t *testing.T) {
	t.Run("Write bytes to file", func(t *testing.T) {
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
	})
}

func TestWritePCDToFile(t *testing.T) {
	t.Run("Write PCD to file", func(t *testing.T) {
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
	})
}

func TestCreateTimestampFilename(t *testing.T) {
	_ = golog.NewTestLogger(t)
	dataDirectory := "/Users/whoami/slam"
	primarySensorName := "myCamera"
	fileType := ".fakepng"
	timeStamp := time.Date(1955, time.March, 13, 01, 10, 30, 00, time.UTC)

	filepathActual := CreateTimestampFilename(dataDirectory, primarySensorName, fileType, timeStamp)
	filepathExpected := "/Users/whoami/slam/myCamera_data_1955-03-13T01:10:30.0000Z.fakepng"
	test.That(t, filepathActual, test.ShouldEqual, filepathExpected)
}
