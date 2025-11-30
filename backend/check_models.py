import os
from pathlib import Path

import google.generativeai as genai
from dotenv import load_dotenv

env_path = Path(__file__).parent / ".env"
load_dotenv(dotenv_path=env_path)

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "").strip().replace('"', "")

if not GEMINI_API_KEY:
    raise ValueError("GEMINI_API_KEY not found in environment variables.")

genai.configure(api_key=GEMINI_API_KEY)

print("Available Gemini Models:")

for m in genai.list_models():
    if "generateContent" in m.supported_generation_methods:
        print(m.name)
