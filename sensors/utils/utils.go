package utils

import (
	"context"
	"image"

	"github.com/edaniels/gostream"
	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/utils"
)

// GetPNGImage first attempts to get a lazy PNG image. If the image is not a lazy PNG, the
// function expects it to be an YCBCR image. If it is neither a lazy PNG nor an YCBCR, the
// function errors out.
func GetPNGImage(ctx context.Context, cam camera.Camera) ([]byte, func(), error) {
	// We will hint that we want a PNG.
	// The Camera service server implementation in RDK respects this; others may not.
	readImgCtx := gostream.WithMIMETypeHint(ctx, utils.WithLazyMIMEType(utils.MimeTypePNG))
	img, release, err := camera.ReadImage(readImgCtx, cam)
	if err != nil {
		return nil, nil, err
	}
	if lazyImg, ok := img.(*rimage.LazyEncodedImage); ok {
		if lazyImg.MIMEType() != utils.MimeTypePNG {
			return nil, nil, errors.Errorf("expected mime type %v, got %T", utils.MimeTypePNG, img)
		}
		return lazyImg.RawData(), release, nil
	}

	if ycbcrImg, ok := img.(*image.YCbCr); ok {
		pngImage, err := rimage.EncodeImage(ctx, ycbcrImg, utils.MimeTypePNG)
		if err != nil {
			return nil, nil, err
		}
		return pngImage, release, nil
	}

	return nil, nil, errors.Errorf("expected lazily encoded image or ycbcrImg, got %T", img)
}
