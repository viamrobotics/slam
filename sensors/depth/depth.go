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
	Name  string
	depth camera.Camera
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

	return Depth{
		Name:  name,
		depth: newDepth,
	}, nil
}

// GetData returns data from the depth sensor.
func (depth Depth) GetData(ctx context.Context) ([]byte, func(), error) {
	return utils.GetPNGImage(ctx, depth.depth)
}
