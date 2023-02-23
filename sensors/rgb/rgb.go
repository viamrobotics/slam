package rgb

import (
	"context"
	"image"

	"go.viam.com/rdk/rimage"

	"github.com/edaniels/gostream"
	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/registry"
	"go.viam.com/rdk/rimage/transform"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/slam/sensors"
)

type RGB struct {
	Name       string
	rgb        camera.Camera
	DataRateMs int
}

func validate(ctx context.Context, rgb camera.Camera) error {
	proj, err := rgb.Projector(ctx)
	if err != nil {
		return errors.Wrap(err,
			"Unable to get camera features for first camera, make sure the color camera is listed first")
	}
	intrinsics, ok := proj.(*transform.PinholeCameraIntrinsics)
	if !ok {
		return transform.NewNoIntrinsicsError("Intrinsics do not exist")
	}

	err = intrinsics.CheckValid()
	if err != nil {
		return err
	}

	props, err := rgb.Properties(ctx)
	if err != nil {
		return errors.Wrap(err, "error getting camera properties for slam service")
	}

	brownConrady, ok := props.DistortionParams.(*transform.BrownConrady)
	if !ok {
		return errors.New("error getting distortion_parameters for slam service, only BrownConrady distortion parameters are supported")
	}
	if err := brownConrady.CheckValid(); err != nil {
		return errors.Wrapf(err, "error validating distortion_parameters for slam service")

	}
	return nil
}

func (rgb RGB) GetData(ctx context.Context) ([]byte, func(), error) {
	return GetPNGImage(ctx, rgb.rgb)
}

func GetPNGImage(ctx context.Context, rgbCamera camera.Camera) ([]byte, func(), error) {
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

func New(ctx context.Context, deps registry.Dependencies, sensor sensors.Sensor, name string) (RGB, error) {
	newRGB, err := camera.FromDependencies(deps, name)
	if err != nil {
		return RGB{}, errors.Wrapf(err, "error getting camera %v for slam service", name)
	}

	if err = validate(ctx, newRGB); err != nil {
		return RGB{}, err
	}

	// TODO: This is where we'd calculate/set the datarate
	dataRateMs := 0

	return RGB{
		Name:       name,
		rgb:        newRGB,
		DataRateMs: dataRateMs,
	}, nil
}
