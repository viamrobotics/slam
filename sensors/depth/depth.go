package depth

import (
	"context"
	"image"

	"go.viam.com/rdk/rimage"

	"github.com/edaniels/gostream"
	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/registry"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/slam/sensors"
)

type Depth struct {
	depth      camera.Camera
	DataRateMs int
}

func validate(ctx context.Context, rgb camera.Camera) error {
	return nil
}

func (depth Depth) GetData(ctx context.Context) ([]byte, func(), error) {
	return getPNGImage(ctx, depth.depth)
}

func getPNGImage(ctx context.Context, rgbCamera camera.Camera) ([]byte, func(), error) {
	// We will hint that we want a PNG.
	// The Camera service server implementation in RDK respects this; others may not.
	readImgCtx := gostream.WithMIMETypeHint(ctx, rdkutils.WithLazyMIMEType(rdkutils.MimeTypePNG))
	img, release, err := camera.ReadImage(readImgCtx, rgbCamera)
	if err != nil {
		return nil, nil, err
	}
	if lazyImg, ok := img.(*rimage.LazyEncodedImage); ok {
		if lazyImg.MIMEType() != rdkutils.MimeTypePNG {
			return nil, nil, errors.Errorf("expected mime type %v, got %T", rdkutils.MimeTypePNG, img)
		}
		return lazyImg.RawData(), release, nil
	}

	if ycbcrImg, ok := img.(*image.YCbCr); ok {
		pngImage, err := rimage.EncodeImage(ctx, ycbcrImg, rdkutils.MimeTypePNG)
		if err != nil {
			return nil, nil, err
		}
		return pngImage, release, nil
	}

	return nil, nil, errors.Errorf("expected lazily encoded image or ycbcrImg, got %T", img)
}

func New(ctx context.Context, deps registry.Dependencies, sensor sensors.Sensor, name string) (Depth, error) {
	newDepth, err := camera.FromDependencies(deps, name)
	if err != nil {
		return Depth{}, errors.Wrapf(err, "error getting camera %v for slam service", name)
	}

	if err = validate(ctx, newDepth); err != nil {
		return Depth{}, err
	}

	dataRateMs := 0

	return Depth{
		depth:      newDepth,
		DataRateMs: dataRateMs,
	}, nil
}
