#!/usr/bin/env python

import yaml

with open("hud.yaml", 'r') as stream:
    try:
        text = yaml.safe_load(stream)

        referenceFrame = text['points']['TL'][0]

        TL = text['points']['TL'][1]
        TR = text['points']['TR'][1]
        BR = text['points']['BR'][1]
        BL = text['points']['BL'][1]

        for entry in text['segments']:
            point1 = entry['points'][0]
            point2 = entry['points'][1]
            color = entry['color']

    except yaml.YAMLError as exc:
        print(exc)
