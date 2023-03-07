package sensors

import (
	"github.com/pkg/errors"
	"go.viam.com/slam/config"
)

type Sensor struct {
	Id                string
	Index             int
	DefaultDataRateMs int
}

func (sensor Sensor) GetName(svcConfig *config.AttrConfig) (string, error) {
	if sensor.Index > len(svcConfig.Sensors) {
		return "", errors.New("index out of bounds")
	}
	return svcConfig.Sensors[sensor.Index], nil
}

func (sensor Sensor) GetDataRateMs(svcConfig *config.AttrConfig) int {
	if svcConfig.DataRateMsec == 0 {
		return sensor.DefaultDataRateMs
	}
	return svcConfig.DataRateMsec
}
