// Package sensors implements the high level structure of how sensors
// are defined in slam libraries
package sensors

import (
	"github.com/pkg/errors"

	"go.viam.com/slam/config"
)

// Sensor contains the necessary information to define a generic sensor.
type Sensor struct {
	ID                string
	Index             int
	DefaultDataRateMs int
}

// GetName returns the name of the sensor based on its index in the sensor array.
func (sensor Sensor) GetName(svcConfig *config.AttrConfig) (string, error) {
	if sensor.Index > len(svcConfig.Sensors) {
		return "", errors.New("index out of bounds")
	}
	return svcConfig.Sensors[sensor.Index], nil
}

// GetDataRateMsec returns the data rate based on the sensors preferences and
// the service config.
func (sensor Sensor) GetDataRateMsec(svcConfig *config.AttrConfig) int {
	if svcConfig.DataRateMsec == 0 {
		return sensor.DefaultDataRateMs
	}
	return svcConfig.DataRateMsec
}
