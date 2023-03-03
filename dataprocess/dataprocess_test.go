package dataprocess

import (
	"testing"
    "os"
    "image"
	"github.com/edaniels/golog"
	"go.viam.com/test"
)

func TestWriteImage(t *testing.T) {
	logger := golog.NewTestLogger(t)
    tempDir, err := os.MkdirTemp();
    test.That(t, err, test.ShouldBeNil)
    img := image.NewRGBA(image.Rectangle{
        image.Point{0, 0},
        image.Point{200,200}
    })



}

