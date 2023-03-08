// Package utils contains helper functions for the slam library implementations
package utils

import "strings"

// DictToString converts a dictionary to a string so
// it can be loaded into an arg for the slam process.
func DictToString(m map[string]string) string {
	stringMapList := make([]string, len(m))
	i := 0
	for k, val := range m {
		stringMapList[i] = k + "=" + val
		i++
	}
	stringMap := strings.Join(stringMapList, ",")

	return "{" + stringMap + "}"
}
