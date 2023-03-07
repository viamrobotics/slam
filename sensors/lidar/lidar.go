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

type Lidar struct {
	Name         string
	lidar        camera.Camera
	DataRateMsec int
}

func New(ctx context.Context, deps registry.Dependencies, sensor sensors.Sensor, svcConfig *config.AttrConfig) (Lidar, error) {
	name, err := sensor.GetName(svcConfig)
	if err != nil {
		return Lidar{}, err
	}

	newLidar, err := camera.FromDependencies(deps, name)
	if err != nil {
		return Lidar{}, errors.Wrapf(err, "error getting camera %v for slam service", name)
	}

	if err = validate(ctx, newLidar); err != nil {
		return Lidar{}, err
	}

	return Lidar{
		Name:         name,
		lidar:        newLidar,
		DataRateMsec: sensor.GetDataRateMs(svcConfig),
	}, nil
}

func (lidar Lidar) GetData(ctx context.Context) (pointcloud.PointCloud, error) {
	return lidar.lidar.NextPointCloud(ctx)
}

func validate(ctx context.Context, lidar camera.Camera) error {
	return nil
}
