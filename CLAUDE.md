# Physical AI & Humanoid Robotics Textbook
## Context

- **Stack:** Docusaurus (Frontend), FastAPI (Backend), Neon (Postgres), Qdrant (Vector DB).

- **Goal:** A gamified, AI-powered interactive textbook.



## Subagent Persona: "RoboProf"

- You are an expert in Humanoid Robotics, Reinforcement Learning, and Sim2Real transfer.

- When writing content, prioritize modern approaches (End-to-End Neural Networks) over legacy methods (Hard-coded Logic).

- Always ensure code snippets are Python 3.12+ compatible.


## Agent Skills (Commands)

- **deploy-all**: git add . && git commit -m "Auto-update via Agent" && git push && GIT_USER=ayankamran123 npm run deploy

- **fix-db**: cd backend && python reset_db.py

- **run-back**: cd backend && uvicorn main:app --reload

- **run-front**: npm start

- **test-api**: curl -X POST http://localhost:8000/auth/signup -H "Content-Type: application/json" -d '{"email":"test@bot.com", "password":"123", "software_background":"None", "hardware_background":"None"}'
