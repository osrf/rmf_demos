import glob, os
import json
from pathlib import Path

app_config = {
    "dispensers":{},
    "robots":{}
}

for topic in app_config.keys():    
    files = Path(topic + '/').glob("**/*.json")
    for file in files:
        with open(file) as json_file:
            print(file)
            app_config[topic].update(json.load(json_file)) 
    

with open('main.json', 'w') as main_file:
    print(app_config)
    main_file.write(json.dumps(app_config))
