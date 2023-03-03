// Package dataprocess manages code related to the data-accepting process
package dataprocess

import (
	"bufio"
	"bytes"
	"context"
	"image"
	"image/png"
	"os"

	"github.com/pkg/errors"
	pc "go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/rimage"
	rdkutils "go.viam.com/rdk/utils"
)

func convertImageToPNG(ctx context.Context, img image.Image) ([]byte, error) {
	if lazyImg, ok := img.(*rimage.LazyEncodedImage); ok {
		if lazyImg.MIMEType() != rdkutils.MimeTypePNG {
			return nil, errors.Errorf("expected mime type %v, got %T", rdkutils.MimeTypePNG, img)
		}
		return lazyImg.RawData(), nil
	}

	if ycbcrImg, ok := img.(*image.YCbCr); ok {
		pngImage, err := rimage.EncodeImage(ctx, ycbcrImg, rdkutils.MimeTypePNG)
		if err != nil {
			return nil, err
		}
		return pngImage, nil
	}
	buf := new(bytes.Buffer)
	err := png.Encode(buf, img)
	if err == nil {
		return buf.Bytes(), nil
	}
	return nil, errors.New("Failed to convert image to PNG")
}

// WriteImageToPNGFile attempts to convert an image to PNG and then save it to the passed filename.
func WriteImageToPNGFile(ctx context.Context, image image.Image, filename string) error {
	png, err := convertImageToPNG(ctx, image)
	if err != nil {
		return err
	}
	//nolint:gosec
	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	w := bufio.NewWriter(f)
	if _, err := w.Write(png); err != nil {
		return err
	}
	if err := w.Flush(); err != nil {
		return err
	}
	return f.Close()
}

// WritePCDToFile Encodes the pointcloud and then saves it to the passed filename.
func WritePCDToFile(ctx context.Context, pointcloud pc.PointCloud, filename string) error {
	//nolint:gosec
	f, err := os.Create(filename)
	if err != nil {
		return err
	}

	w := bufio.NewWriter(f)

	if err = pc.ToPCD(pointcloud, w, 1); err != nil {
		return err
	}
	if err = w.Flush(); err != nil {
		return err
	}
	return f.Close()
}
