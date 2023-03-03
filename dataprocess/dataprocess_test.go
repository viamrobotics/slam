package dataprocess

import (
	"context"
	"image"
	"os"
	"testing"

	"github.com/edaniels/golog"
	pc "go.viam.com/rdk/pointcloud"
	"go.viam.com/test"
)

func TestWriteImage(t *testing.T) {
	_ = golog.NewTestLogger(t)
	tempDir, err := os.MkdirTemp("", "*")
	defer os.RemoveAll(tempDir)
	test.That(t, err, test.ShouldBeNil)
	img := image.NewRGBA(image.Rectangle{
		image.Point{0, 0},
		image.Point{200, 200},
	})
	fileDest := tempDir + "test_img.png"
	err = WriteImageToPNGFile(context.Background(), img, fileDest)
	test.That(t, err, test.ShouldBeNil)
	// Test that the file was actually written
	_, err = os.Stat(fileDest)
	test.That(t, err, test.ShouldBeNil)
}

func TestWritePCD(t *testing.T) {
	_ = golog.NewTestLogger(t)
	tempDir, err := os.MkdirTemp("", "*")
	defer os.RemoveAll(tempDir)
	test.That(t, err, test.ShouldBeNil)
	fileDest := tempDir + "test_pcd.pcd"
	pointcloud := pc.New()
	err = WritePCDToFile(context.Background(), pointcloud, fileDest)
	test.That(t, err, test.ShouldBeNil)
	// Test that the file was actually written
	_, err = os.Stat(fileDest)
	test.That(t, err, test.ShouldBeNil)
}
