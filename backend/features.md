# Bonus Features Specification

## Project Context
- **Frontend:** Docusaurus (React) deployed on GitHub Pages.
- **Backend:** FastAPI with Neon (Serverless Postgres) and Qdrant.
- **Goal:** Gamify the book with Auth, Personalization, and Localization.

## Feature 1: Authentication & User Profiling
- **Requirement:** Implement a Signup and Signin flow.
- **Technology:** Use Better-Auth logic (or a secure JWT Python equivalent if Better-Auth is Node-only).
- **Data Schema:** The User table in Neon Postgres MUST store:
  - `id`, `email`, `hashed_password`
  - `software_background` (string)
  - `hardware_background` (string)
- **User Flow:**
  - When a user signs up, ask two mandatory questions:
    1. "What is your software development background?"
    2. "What is your hardware/IoT background?"
  - Save these answers to the database.

## Feature 2: Content Personalization Button
- **Requirement:** A "✨ Personalize" button at the start of every chapter.
- **Constraint:** Only visible to logged-in users.
- **Workflow:**
  1. User clicks the button.
  2. Frontend sends `current_page_text` and `user_token` to Backend `/api/personalize`.
  3. Backend fetches user's `software_background` and `hardware_background` from DB.
  4. Backend prompts the LLM: "Rewrite this text for someone with [Software BG] and [Hardware BG]. Use relevant analogies."
  5. Frontend replaces the static text with the personalized response.
## Feature 3: Urdu Translation Button
- **Requirement:** A "🌐 Translate to Urdu" button at the start of every chapter.
- **Workflow:**
  1. User clicks the button.
  2. Frontend sends `current_page_text` to Backend `/api/translate`.
  3. Backend prompts the LLM: "Translate this technical content to Urdu."
  4. Frontend replaces the English text with Urdu.
