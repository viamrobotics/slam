// Package rgb implements the RGB sensor
package rgb

import (
	"context"

	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/registry"
	"go.viam.com/rdk/rimage/transform"

	"go.viam.com/slam/config"
	"go.viam.com/slam/sensors"
	"go.viam.com/slam/sensors/utils"
)

// RGB represents an RGB sensor.
type RGB struct {
	Name         string
	rgb          camera.Camera
	DataRateMsec int
}

// New creates a new RGB sensor based on the sensor definition and the service config.
func New(ctx context.Context, deps registry.Dependencies, sensor sensors.Sensor, svcConfig *config.AttrConfig) (RGB, error) {
	name, err := sensor.GetName(svcConfig)
	if err != nil {
		return RGB{}, err
	}

	newRGB, err := camera.FromDependencies(deps, name)
	if err != nil {
		return RGB{}, errors.Wrapf(err, "error getting camera %v for slam service", name)
	}

	if err = validate(ctx, newRGB); err != nil {
		return RGB{}, err
	}

	return RGB{
		Name:         name,
		rgb:          newRGB,
		DataRateMsec: sensor.GetDataRateMsec(svcConfig),
	}, nil
}

// GetData returns data from the RGB sensor.
func (rgb RGB) GetData(ctx context.Context) ([]byte, func(), error) {
	return utils.GetPNGImage(ctx, rgb.rgb)
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
