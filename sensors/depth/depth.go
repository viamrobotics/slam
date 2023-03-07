// Package depth implements the Depth sensor
package depth

import (
	"context"

	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/registry"

	"go.viam.com/slam/config"
	"go.viam.com/slam/sensors"
	"go.viam.com/slam/sensors/utils"
)

// Depth represents a depth sensor.
type Depth struct {
	Name         string
	depth        camera.Camera
	DataRateMsec int
}

// New creates a new Depth sensor based on the sensor definition and the service config.
func New(ctx context.Context, deps registry.Dependencies, sensor sensors.Sensor, svcConfig *config.AttrConfig) (Depth, error) {
	name, err := sensor.GetName(svcConfig)
	if err != nil {
		return Depth{}, err
	}

	newDepth, err := camera.FromDependencies(deps, name)
	if err != nil {
		return Depth{}, errors.Wrapf(err, "error getting camera %v for slam service", name)
	}

	if err = validate(ctx, newDepth); err != nil {
		return Depth{}, err
	}

	return Depth{
		Name:         name,
		depth:        newDepth,
		DataRateMsec: sensor.GetDataRateMsec(svcConfig),
	}, nil
}

// GetData returns data from the depth sensor.
func (depth Depth) GetData(ctx context.Context) ([]byte, func(), error) {
	return utils.GetPNGImage(ctx, depth.depth)
}

func validate(ctx context.Context, rgb camera.Camera) error {
	return nil
}
