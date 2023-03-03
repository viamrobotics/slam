package dataprocess

import (
	"context"
	"image"
	"image/color"
	"image/png"
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
	img := image.NewNRGBA(image.Rectangle{
		image.Point{0, 0},
		image.Point{10, 10},
	})
	img.Set(1, 1, color.Black)
	fileDest := tempDir + "test_img.png"
	err = WriteImageToPNGFile(context.Background(), img, fileDest)
	test.That(t, err, test.ShouldBeNil)
	reader, err := os.Open(fileDest)
	// Test that the file was actually written
	test.That(t, err, test.ShouldBeNil)
	// Test that decoding the image produces the same image
	readImage, err := png.Decode(reader)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, img, test.ShouldResemble, readImage)
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
