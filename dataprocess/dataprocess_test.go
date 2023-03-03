package dataprocess

import (
	"context"
	"image"
	"os"
	"testing"

	"github.com/edaniels/golog"
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
	imgDest := tempDir + "test_img.png"
	err = WriteImageToPNGFile(context.Background(), img, imgDest)
	test.That(t, err, test.ShouldBeNil)
	// Test that the file was actually written
	_, err = os.Stat(imgDest)
	test.That(t, err, test.ShouldBeNil)
}
