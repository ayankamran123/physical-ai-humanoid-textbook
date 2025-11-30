---
id: 0001
title: Rewrite CLAUDE.md with Subagent Persona and Agent Skills
stage: general
date: 2025-11-30
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: main
user: ayan
command: /sp.phr
labels: ["claudemd", "subagent-persona", "agent-skills", "project-context"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - /home/ayan/PIAIC_STUDENT/book/CLAUDE.md
tests:
  - none
---

## Prompt

I need to implement "Reusable Intelligence" and "Agent Skills" to earn the 50 bonus points for my project.



Please completely REWRITE the `CLAUDE.md` file in the root directory. It must follow the Spec-Kit Plus standard and include the following advanced features:



1.  **Subagent Persona (The Brain):**

    - Define a system persona named **"RoboProf"**.

    - Tone: Senior Robotics Engineer at Tesla/Boston Dynamics.

    - Style: Technical, focuses on Sim2Real, uses analogies involving sensors/actuators.



2.  **Agent Skills (The Tools):**

    - Define these reusable commands in the `commands` section:

        - `deploy-all`: A single command to add changes, commit, push to GitHub, AND deploy to GitHub Pages.
- `fix-db`: A command to run the `backend/reset_db.py` script.

        - `run-back`: Command to start the FastAPI backend.

        - `run-front`: Command to start Docusaurus.

        - `new-chapter`: A command to create a new markdown file in `docs/` with standard frontmatter.



3.  **Project Context:**

    - Briefly mention the stack: Docusaurus, FastAPI, Neon, Qdrant.



Here is the exact structure I want for `CLAUDE.md`. Please apply it:



```markdown

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
```

## Response snapshot

I have rewritten the `CLAUDE.md` file with the specified subagent persona, agent skills, and project context.

## Outcome

- ✅ Impact: The `CLAUDE.md` file was successfully rewritten to incorporate the 'RoboProf' subagent persona, custom agent skills, and project context as per the user's requirements. This updates the agent's operating instructions and available commands, enhancing its capabilities within the Spec-Kit Plus standard.
- 🧪 Tests: none
- 📁 Files: One file was modified: `/home/ayan/PIAIC_STUDENT/book/CLAUDE.md`
- 🔁 Next prompts: The user might want to verify the changes in `CLAUDE.md` or try out the newly defined agent skills.
- 🧠 Reflection: Successfully followed the multi-step instruction to first perform a file rewrite and then create a PHR for the interaction, demonstrating adherence to documentation protocols.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): none
