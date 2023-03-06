package dataprocess

import (
	"bytes"
	"context"
	"image"
	"image/png"
	"os"
	"testing"

	"github.com/edaniels/golog"
	pc "go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/rimage"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/test"
)

func TestWriteImage(t *testing.T) {
	_ = golog.NewTestLogger(t)
	tempDir, err := os.MkdirTemp("", "*")
	defer os.RemoveAll(tempDir)
	test.That(t, err, test.ShouldBeNil)
	// Create an image to encode
	origImg := image.NewRGBA(image.Rectangle{
		image.Point{0, 0},
		image.Point{10, 10},
	})
	buf := new(bytes.Buffer)
	png.Encode(buf, origImg)
	// Encode it as a LazyImage
	lazyImg := rimage.NewLazyEncodedImage(buf.Bytes(), rdkutils.MimeTypePNG)
	// Save the encoded image
	ctx := context.Background()
	fileDest := tempDir + "test_img.png"
	err = WriteImageToPNGFile(ctx, lazyImg, fileDest)
	test.That(t, err, test.ShouldBeNil)
	// Test that the file was actually written
	imgBytes, err := os.ReadFile(fileDest)
	test.That(t, err, test.ShouldBeNil)
	// Test that decoding the image produces the same image
	readImg := rimage.NewLazyEncodedImage(imgBytes, rdkutils.MimeTypePNG)
	test.That(t, readImg, test.ShouldResemble, lazyImg)
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
