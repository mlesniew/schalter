#!/usr/bin/env bash

cd "$(dirname $0)"

rm -rf data/html
cp -r html data/
gzip -9 data/html/*
