// Package lidar implements the Lidar sensor
package lidar

import (
	"context"

	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/registry"

	"go.viam.com/slam/config"
	"go.viam.com/slam/sensors"
)

// Lidar represents a LIDAR sensor.
type Lidar struct {
	Name  string
	lidar camera.Camera
}

// New creates a new Lidar sensor based on the sensor definition and the service config.
func New(ctx context.Context, deps registry.Dependencies, sensor sensors.Sensor, svcConfig *config.AttrConfig) (Lidar, error) {
	name, err := sensor.GetName(svcConfig)
	if err != nil {
		return Lidar{}, err
	}

	newLidar, err := camera.FromDependencies(deps, name)
	if err != nil {
		return Lidar{}, errors.Wrapf(err, "error getting camera %v for slam service", name)
	}

	return Lidar{
		Name:  name,
		lidar: newLidar,
	}, nil
}

// GetData returns data from the lidar sensor.
func (lidar Lidar) GetData(ctx context.Context) (pointcloud.PointCloud, error) {
	return lidar.lidar.NextPointCloud(ctx)
}
