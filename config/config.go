// Package config implements functions to assist with attribute evaluation in the SLAM service
package config

import (
	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"go.viam.com/rdk/config"
	"go.viam.com/utils"
)

// newError returns an error specific to a failure in the SLAM config.
func newError(configError string) error {
	return errors.Errorf("SLAM Service configuration error: %s", configError)
}

// DetermineDeleteProcessedData will determine the value of the deleteProcessData attribute
// based on the useLiveData and deleteData input parameters.
func DetermineDeleteProcessedData(logger golog.Logger, deleteData *bool, useLiveData bool) bool {
	var deleteProcessedData bool
	if deleteData == nil {
		deleteProcessedData = useLiveData
	} else {
		deleteProcessedData = *deleteData
		if !useLiveData && deleteProcessedData {
			logger.Debug("a value of true cannot be given for delete_processed_data when in offline mode, setting to false")
			deleteProcessedData = false
		}
	}
	return deleteProcessedData
}

// DetermineUseLiveData will determine the value of the useLiveData attribute
// based on the liveData input parameter and sensor list.
func DetermineUseLiveData(logger golog.Logger, liveData *bool, sensors []string) (bool, error) {
	if liveData == nil {
		return false, newError("use_live_data is a required input parameter")
	}
	useLiveData := *liveData
	if useLiveData && len(sensors) == 0 {
		return false, newError("sensors field cannot be empty when use_live_data is set to true")
	}
	return useLiveData, nil
}

// AttrConfig describes how to configure the SLAM service.
type AttrConfig struct {
	Sensors             []string          `json:"sensors"`
	ConfigParams        map[string]string `json:"config_params"`
	DataDirectory       string            `json:"data_dir"`
	UseLiveData         *bool             `json:"use_live_data"`
	DataRateMsec        int               `json:"data_rate_msec"`
	MapRateSec          *int              `json:"map_rate_sec"`
	Port                string            `json:"port"`
	DeleteProcessedData *bool             `json:"delete_processed_data"`
	Dev                 bool              `json:"dev"`
}

// NewAttrConfig creates a SLAM config from a service config.
func NewAttrConfig(cfg config.Service) (*AttrConfig, error) {
	attrCfg := &AttrConfig{}

	_, err := config.TransformAttributeMapToStruct(attrCfg, cfg.Attributes)
	if err != nil {
		return &AttrConfig{}, newError(err.Error())
	}

	// TODO Replace this config_path with a more sensible value
	_, err = attrCfg.Validate("config_path")
	if err != nil {
		return &AttrConfig{}, newError(err.Error())
	}

	return attrCfg, nil
}

// Validate creates the list of implicit dependencies.
func (config *AttrConfig) Validate(path string) ([]string, error) {
	if config.ConfigParams["mode"] == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "config_params[mode]")
	}

	if config.DataDirectory == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "data_dir")
	}

	if config.UseLiveData == nil {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "use_live_data")
	}

	if config.DataRateMsec < 0 {
		return nil, errors.New("cannot specify data_rate_msec less than zero")
	}

	if config.MapRateSec != nil && *config.MapRateSec < 0 {
		return nil, errors.New("cannot specify map_rate_sec less than zero")
	}

	deps := config.Sensors

	return deps, nil
}

// SetOptionalParameters updates any unset optional config parameters to the values passed to this function.
func (config *AttrConfig) SetOptionalParameters(port string, defaultDataRateMsec, defaultMapRateSec int, logger golog.Logger) error {
	if config.Port == "" {
		config.Port = port
	}

	if config.DataRateMsec == 0 {
		config.DataRateMsec = defaultDataRateMsec
		logger.Debugf("no data_rate_msec given, setting to default value of %d", defaultDataRateMsec)
	}

	if config.MapRateSec == nil {
		logger.Debugf("no map_rate_sec given, setting to default value of %d", defaultMapRateSec)
		config.MapRateSec = &defaultMapRateSec
	}
	if *config.MapRateSec == 0 {
		logger.Info("setting slam system to localization mode")
	}

	useLiveData, err := DetermineUseLiveData(logger, config.UseLiveData, config.Sensors)
	if err != nil {
		return err
	}
	config.UseLiveData = &useLiveData

	deleteProcessedData := DetermineDeleteProcessedData(logger, config.DeleteProcessedData, useLiveData)
	config.DeleteProcessedData = &deleteProcessedData

	return nil
}
