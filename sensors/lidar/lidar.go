package lidar

import (
	"context"

	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/registry"
	slamConfig "go.viam.com/slam/config"
	"go.viam.com/slam/sensors"
)

type Lidar struct {
	Name       string
	lidar      camera.Camera
	DataRateMs int
}

func validate(ctx context.Context, lidar camera.Camera) error {
	return nil
}

func (lidar Lidar) GetData(ctx context.Context) (pointcloud.PointCloud, error) {
	return lidar.lidar.NextPointCloud(ctx)
}

func New(ctx context.Context, deps registry.Dependencies, sensor sensors.Sensor, svcConfig *slamConfig.AttrConfig) (Lidar, error) {
	name, err := sensor.GetName(ctx, svcConfig)
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

	dataRateMs, err := sensor.GetDataRateMs(ctx, svcConfig)
	if err != nil {
		return Lidar{}, err
	}

	return Lidar{
		Name:       name,
		lidar:      newLidar,
		DataRateMs: dataRateMs,
	}, nil
}
