package utils

import (
	"sort"
	"strings"
	"testing"

	"go.viam.com/test"
)

func TestDictToString(t *testing.T) {
	t.Run("Convert dictionay to a string", func(t *testing.T) {
		configParamsDict := map[string]string{
			"min_range": "0.3",
			"max_range": "12",
			"debug":     "false",
			"mode":      "2d",
		}

		expectedConfigParamsString := "{debug=false,mode=2d,min_range=0.3,max_range=12}"
		actualConfigParamsString := DictToString(configParamsDict)
		test.That(t, actualConfigParamsString[0], test.ShouldEqual, expectedConfigParamsString[0])

		expectedLastLetter := expectedConfigParamsString[len(expectedConfigParamsString)-1:]
		actualLastLetter := actualConfigParamsString[len(actualConfigParamsString)-1:]
		test.That(t, actualLastLetter, test.ShouldEqual, expectedLastLetter)

		expectedContents := strings.Split(expectedConfigParamsString[1:len(expectedConfigParamsString)-1], ",")
		actualContents := strings.Split(actualConfigParamsString[1:len(actualConfigParamsString)-1], ",")
		sort.Strings(actualContents)
		sort.Strings(expectedContents)
		test.That(t, actualContents, test.ShouldResemble, expectedContents)
	})
}
