package sensors

import (
	"context"

	"github.com/pkg/errors"
	slamConfig "go.viam.com/slam/config"
)

type Sensor struct {
	Id                string
	DefaultDataRateMs int
	MinDataRateMs     *int
	MaxDataRateMs     *int
}

func (sensor Sensor) GetName(ctx context.Context, svcConfig *slamConfig.AttrConfig) (string, error) {
	name, ok := svcConfig.Sensors[sensor.Id]
	if !ok {
		return "", errors.Errorf("can't find sensor key-value pair %q", sensor.id)
	}
	return name, nil
}

func (sensor Sensor) GetDataRateMs(ctx context.Context, svcConfig *slamConfig.AttrConfig) (int, error) {
	if svcConfig.DataRateMs == 0 {
		return sensor.DefaultDataRateMs, nil
	}
	if sensor.MinDataRateMs != nil && svcConfig.DataRateMs < *sensor.MinDataRateMs {
		return 0, errors.Errorf("cannot specify data_rate_msec less than %v", sensor.MinDataRateMs)
	}
	if sensor.MaxDataRateMs != nil && svcConfig.DataRateMs > *sensor.MaxDataRateMs {
		return 0, errors.Errorf("cannot specify data_rate_msec larger than %v", sensor.MaxDataRateMs)
	}
	return svcConfig.DataRateMs, nil
}
