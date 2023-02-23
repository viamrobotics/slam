// Package config implements functions to assist with attribute evaluation in the slam service
package config

import (

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"go.viam.com/rdk/config"
	"go.viam.com/utils"
)

// NewError returns an error specific to a failure in the SLAM config.
func NewError(configError string) error {
	return errors.Errorf("SLAM Service configuration error: %s", configError)
}

// WrapError wraps an error to show it came from the slam service.
func WrapError(configError error) error {
	return NewError(configError.Error())
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
		return false, NewError("use_live_data is a required input parameter")
	}
	useLiveData := *liveData
	if useLiveData && len(sensors) == 0 {
		return false, NewError("sensors field cannot be empty when use_live_data is set to true")
	}
	return useLiveData, nil
}

// AttrConfig describes how to configure the service.
type AttrConfig struct {
	Sensors             []string          `json:"sensors"`
	ConfigParams        map[string]string `json:"config_params"`
	DataDirectory       string            `json:"data_dir"`
	UseLiveData         *bool             `json:"use_live_data"`
	DataRateMs          int               `json:"data_rate_msec"`
	MapRateSec          *int              `json:"map_rate_sec"`
	InputFilePattern    string            `json:"input_file_pattern"`
	Port                string            `json:"port"`
	DeleteProcessedData *bool             `json:"delete_processed_data"`
	Dev                 bool              `json:"dev"`
}


func NewAttrConfig(cfg config.Service) (returnValue *AttrConfig, returnError error) {

	attrCfg := &AttrConfig{}
    
    _,err := config.TransformAttributeMapToStruct(attrCfg, cfg.Attributes)
	if err != nil {
		return &AttrConfig{}, WrapError(err)
	}

    // TODO Replace this config_path with a more sensible value
	_, err = attrCfg.Validate("config_path")
	if err != nil {
		return &AttrConfig{}, WrapError(err)
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

	if config.DeleteProcessedData == nil {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "delete_processed_data")
	}

	if config.DataRateMs != 0 && config.DataRateMs < 0 {
		return nil, errors.New("cannot specify data_rate_msec less than zero")
	}

	if config.MapRateSec != nil && *config.MapRateSec < 0 {
		return nil, errors.New("cannot specify map_rate_sec less than zero")
	}

	deps := config.Sensors

	return deps, nil
}


// SetParameters ...
func (config *AttrConfig) SetParameters(localhost0 string, defaultDataRateMs, defaultMapRateSec int, logger golog.Logger) error {

	if config.Port == "" {
		config.Port = localhost0
	}

	if config.DataRateMs == 0 {
		config.DataRateMs = defaultDataRateMs
		logger.Debugf("no data_rate_msec given, setting to default value of %d", defaultDataRateMs)
	}

	if config.MapRateSec == nil {
		logger.Debugf("no map_rate_secs given, setting to default value of %d", defaultMapRateSec)
		config.MapRateSec = &defaultMapRateSec
	} else if *config.MapRateSec == 0 {
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
