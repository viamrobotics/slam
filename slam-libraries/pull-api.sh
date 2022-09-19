#!/bin/bash

# If it already exists, just update
if [ -d api/ ]
then
	cd api
	git pull origin main
else
	mkdir api
	cd api
	git init -b main
	git remote add origin git@github.com:viamrobotics/api.git
	git config pull.ff only
	git fetch --depth=1 origin main
	git sparse-checkout set --cone "grpc/cpp" "proto" "buf.yaml" "buf.lock" "go.mod" "go.sum"
	git pull origin main
fi


