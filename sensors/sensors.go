package sensors

import (
	"context"

	"github.com/pkg/errors"
	slamConfig "go.viam.com/slam/config"
)

type Sensor struct {
	Id                string
	Index             int
	DefaultDataRateMs int
}

func (sensor Sensor) GetName(ctx context.Context, svcConfig *slamConfig.AttrConfig) (string, error) {
	if sensor.Index > len(svcConfig.Sensors) {
		return "", errors.New("index out of bounds")
	}
	return svcConfig.Sensors[sensor.Index], nil
}

func (sensor Sensor) GetDataRateMs(ctx context.Context, svcConfig *slamConfig.AttrConfig) int {
	if svcConfig.DataRateMs == 0 {
		return sensor.DefaultDataRateMs
	}
	return svcConfig.DataRateMs
}
