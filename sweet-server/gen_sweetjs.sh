#!/bin/sh
protoc --js_out=import_style=commonjs,binary:. sweet.proto
mv sweet_pb.js /mnt/server/sfcloud/sfspice/

