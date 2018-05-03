#! /bin/bash

if [ $# -ne 3 ]
then
	echo "ERROR, expecting arguments: <input.laz> <subsample-step> <output.laz>" >&2
	exit 1
fi

json=$(mktemp --suffix .json)

echo "{
\"pipeline\": [
    {
        \"type\": \"readers.las\",
        \"filename\": \"$1\"
    },
    {
        \"type\":\"filters.decimation\",
        \"step\": $2
    },
    {
      \"type\":\"writers.las\",
      \"filename\":\"$3\"
    }
  ]
}" > $json

echo "Running:" >&2
cat $json >&2

pdal pipeline -i $json

rm $json
