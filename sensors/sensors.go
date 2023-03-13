// Package sensors implements the high level structure of how sensors
// are defined in slam libraries
package sensors

import (
	"github.com/pkg/errors"

	"go.viam.com/slam/config"
)

// Sensor contains the necessary information to define a generic sensor.
type Sensor struct {
	Index int
}

// GetName returns the name of the sensor based on its index in the sensor array.
func (sensor Sensor) GetName(svcConfig *config.AttrConfig) (string, error) {
	if sensor.Index > len(svcConfig.Sensors) {
		return "", errors.New("index out of bounds")
	}
	return svcConfig.Sensors[sensor.Index], nil
}
