import base64
import urllib.request
import os
import json

# Professional UML styles - Compact Layout
mermaid_config = {
    "theme": "base",
    "themeVariables": {
        "primaryColor": "#FFFFFF",
        "primaryTextColor": "#000000",
        "primaryBorderColor": "#000000",
        "lineColor": "#000000",
        "secondaryColor": "#F4F4F4",
        "tertiaryColor": "#FFFFFF",
        "fontFamily": "Arial",
        "fontSize": "16px"
    }
}

json_config = base64.b64encode(json.dumps(mermaid_config).encode('utf-8')).decode('ascii')

diagrams = {
    # 4. Activity Diagram - BALANCED (Wider, Shorter)
    "activity_diagram.png": """stateDiagram-v2
    direction TB
    
    state "Initialization Phase" as Init {
        direction LR
        [*] --> CheckPerms
        CheckPerms --> LoadNative
        LoadNative --> GravityAlign
    }

    Init --> Tracking : [Success]

    state "Active Navigation" as Tracking {
        direction LR
        state "Visual Fusion (VIO)" as VIO
        state "Dead Reckoning (DR)" as DR
        
        [*] --> VIO
        VIO --> DR : Lost Features
        DR --> VIO : Recovered
    }

    Tracking --> Paused : User Pause
    Paused --> Tracking : Resume
    Tracking --> [*] : Stop Session"""
}

output_dir = "Final-Project/SDD/assets"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

for filename, mermaid_code in diagrams.items():
    graphbytes = mermaid_code.encode("utf8")
    base64_bytes = base64.urlsafe_b64encode(graphbytes)
    base64_string = base64_bytes.decode("ascii")
    
    url = f"https://mermaid.ink/img/{base64_string}?mermaid={json_config}"
    
    try:
        print(f"Downloading {filename}...")
        req = urllib.request.Request(
            url, 
            data=None, 
            headers={'User-Agent': 'Mozilla/5.0'}
        )
        with urllib.request.urlopen(req) as response, open(os.path.join(output_dir, filename), 'wb') as out_file:
            out_file.write(response.read())
        print(f"Successfully saved {filename}")
    except Exception as e:
        print(f"Failed to download {filename}: {e}")
