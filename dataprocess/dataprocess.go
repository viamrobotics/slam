package dataprocess

import (
    "os"
    "bufio"
    "image"
    "context"
	"github.com/pkg/errors"
	"go.viam.com/rdk/rimage"
	rdkutils "go.viam.com/rdk/utils"
)

func convertImageToPNG(ctx context.Context, img image.Image) ([]byte, error) {
    if lazyImg, ok := img.(*rimage.LazyEncodedImage); ok {
		if lazyImg.MIMEType() != rdkutils.MimeTypePNG {
			return nil, errors.Errorf("expected mime type %v, got %T", rdkutils.MimeTypePNG, img)
		}
		return lazyImg.RawData(),  nil
	}

	if ycbcrImg, ok := img.(*image.YCbCr); ok {
		pngImage, err := rimage.EncodeImage(ctx, ycbcrImg, rdkutils.MimeTypePNG)
		if err != nil {
			return nil,  err
		}
		return pngImage,  nil
	}
    return nil, errors.New("Failed to convert image to PNG")
}

func WriteImageToPNGFile(ctx context.Context,image image.Image, filename string) error {

    png,err := convertImageToPNG(ctx,image)
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
