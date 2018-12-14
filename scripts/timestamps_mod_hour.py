#! /usr/bin/env python

import json
import sys

records = json.load(sys.stdin)

for r in records:
    if r["timeStamp"] > 3600:
        r["timeStamp"] -= 3600

json.dump(records, sys.stdout)
