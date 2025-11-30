# Technical Specification: Physical AI & Humanoid Robotics – Textbook + RAG System

**Project**: Physical AI & Humanoid Robotics – Textbook + RAG System
**Version**: 1.0.0
**Date**: 2025-11-28
**Status**: Ready for Implementation
**Source Constitution**: .specify/memory/constitution.md

---

## 1. Folder Structure

The project will adhere to a clear, modular folder structure to facilitate AI-driven development, maintainability, and clear separation of concerns.

```
.github/
├── workflows/           # GitHub Actions workflows for CI/CD
│   ├── frontend.yml     # Docusaurus build and deploy to GitHub Pages
│   └── backend.yml      # FastAPI build, deploy, and Markdown indexing
backend/                 # FastAPI backend for RAG system
├── app/                 # FastAPI application core
│   ├── main.py          # Main FastAPI application entry point
│   ├── api/             # API routers (e.g., v1/rag.py)
│   ├── schemas/         # Pydantic models for request/response validation
│   ├── services/        # Business logic for Qdrant, embeddings, LLM interaction
│   ├── utils/           # Utility functions (e.g., markdown chunking, text cleaning)
│   └── core/            # Core configurations, logging, exceptions
├── tests/               # Unit and integration tests for backend
├── Dockerfile           # Dockerfile for backend containerization
├── requirements.txt     # Python dependencies
├── .env.example         # Template for environment variables
frontend/                # Docusaurus frontend for the textbook and chat widget
├── docusaurus.config.js # Docusaurus configuration
├── src/
│   ├── components/      # React components (e.g., RAGChatWidget.js/tsx)
│   ├── pages/           # Docusaurus pages (if any, typically docs are primary)
│   └── css/             # Custom CSS for Docusaurus theming
├── docs/                # Markdown files for textbook chapters (AI-generated)
│   ├── intro.md
│   ├── chapter-1-foundations-of-ai.md
│   ├── chapter-2-robot-kinematics.md
│   ├── chapter-3-robot-dynamics.md
│   ├── chapter-4-sensors-and-perception.md
│   ├── chapter-5-actuators-and-control.md
│   ├── chapter-6-motion-planning.md
│   ├── chapter-7-human-robot-interaction.md
│   ├── chapter-8-robot-learning-and-adaptation.md
│   ├── chapter-9-ethical-considerations.md
│   └── references.md
├── static/              # Static assets (images, logos)
├── package.json         # Node.js dependencies
├── yarn.lock            # Yarn lock file
.specify/                # Spec-Kit Plus configuration and templates
├── memory/
│   └── constitution.md  # Project constitution
├── templates/           # Templates for spec, plan, tasks, PHRs, etc.
└── scripts/             # Utility scripts (e.g., create-phr.sh)
history/                 # Project history and records
├── prompts/             # Prompt History Records (PHRs)
└── adr/                 # Architectural Decision Records (ADRs)
README.md                # Project README
```

---

## 2. Book Content Specification

The textbook will be an AI-generated, university-level resource on "Physical AI & Humanoid Robotics," comprising 8-10 chapters.

### Chapter Naming & Structure
- **Format**: `chapter-N-short-title.md` (e.g., `chapter-1-foundations-of-ai.md`). `intro.md` and `references.md` will be exceptions.
- **Content**: Each chapter will include:
    - **Objectives**: Clear learning outcomes for the chapter.
    - **Theory**: Detailed explanations of concepts.
    - **Diagrams**: Conceptual illustrations (as Mermaid diagrams or image placeholders with descriptions for future AI generation/insertion).
    - **Python/ROS2 Code Examples**: Practical code snippets illustrating concepts, embedded as fenced code blocks.
    - **MCQs**: Multiple Choice Questions for self-assessment, using a custom markdown syntax or React component for rendering.

### Example Chapter Outline (8-10 Chapters)
1.  **Introduction & Foundations of Physical AI**: Overview, history, AI paradigms, ethical considerations.
2.  **Robot Kinematics**: Forward/inverse kinematics, Denavit-Hartenberg parameters, Jacobians.
3.  **Robot Dynamics**: Newton-Euler and Lagrange formulations, rigid body dynamics, control torques.
4.  **Sensors and Perception**: Vision, LiDAR, force/torque sensors, state estimation, SLAM.
5.  **Actuators and Control**: Motors, gearboxes, PID control, impedance control, whole-body control.
6.  **Motion Planning & Navigation**: Path planning, trajectory generation, obstacle avoidance, reactive control.
7.  **Human-Robot Interaction**: HRI principles, shared autonomy, safety, human-in-the-loop systems.
8.  **Robot Learning & Adaptation**: Reinforcement learning for robotics, imitation learning, transfer learning.
9.  **Advanced Topics / Case Studies**: Biomimetic robots, soft robotics, ethical AI in physical systems.
10. **References & Future Directions**: Further reading, open challenges, research trends.

### Markdown Formatting and Naming Conventions
- Standard CommonMark Markdown for text.
- Fenced code blocks (` ```python ``, ` ```ros2 ``) for code examples.
- Custom directives or specific syntax for MCQs and diagrams, to be defined in `frontend/docs/`. Example for MCQs:
    ```markdown
    :::quiz
    Which of the following is an example of an inverse kinematics problem?
    - [ ] Determining joint angles from end-effector position
    - [ ] Determining end-effector position from joint angles
    - [x] Both A and B
    - [ ] Neither A nor B
    :::
    ```
- Image placeholders for diagrams will use standard Markdown image syntax `![Alt Text](path/to/image.png)`. AI to generate content and suggest image names/paths.

### Spec-Kit Plus Templates Usage
- The generated book content (markdown files) will be the primary output of the AI.

---

## 3. Backend Specification

The backend will be a FastAPI application responsible for content indexing, search, and RAG-powered chat.

### FastAPI Endpoints

- **`POST /index`**
    - **Description**: Triggers the processing and indexing of markdown content.
    - **Request Body**:
        ```json
        {
          "file_path": "string",  // Path to the markdown file to index (e.g., "frontend/docs/chapter-1.md")
          "chapter_title": "string" // Title of the chapter for metadata
        }
        ```
    - **Response**:
        ```json
        {
          "status": "success",
          "indexed_chunks": 10,  // Number of text chunks indexed
          "message": "Markdown content successfully indexed."
        }
        ```
    - **Error Handling**: Returns `404 Not Found` if `file_path` does not exist, `500 Internal Server Error` on indexing failure.

- **`POST /search`**
    - **Description**: Retrieves relevant text chunks from the Qdrant vector database based on a natural language query.
    - **Request Body**:
        ```json
        {
          "query": "string",     // User's search query
          "limit": 5             // Number of top relevant chunks to retrieve (default 5)
        }
        ```
    - **Response**:
        ```json
        {
          "results": [
            {
              "text": "string",       // Retrieved text chunk content
              "metadata": {           // Metadata associated with the chunk
                "chapter_title": "string",
                "file_path": "string",
                "chunk_id": "string",
                "page_number": "integer"
              },
              "score": 0.876          // Similarity score
            }
          ]
        }
        ```
    - **Error Handling**: Returns `500 Internal Server Error` on Qdrant or embedding service issues.

- **`POST /chat`**
    - **Description**: Facilitates RAG-powered conversations with the LLM. Supports selected-text only mode.
    - **Request Body**:
        ```json
        {
          "user_message": "string",         // The user's question
          "context_text": "string | null"   // Optional: User-selected text for specific context
        }
        ```
    - **Response**:
        ```json
        {
          "chatbot_response": "string",      // The LLM's generated answer
          "sources": ["string"]              // Optional: List of sources (e.g., "Chapter 1: Foundations, page 15")
        }
        ```
    - **Logic**:
        - If `context_text` is provided, the RAG system *must* strictly use this text as the sole context for the LLM.
        - If `context_text` is `null`, the `services.search` (internally calls `/search`) will be invoked with `user_message` to retrieve relevant chunks from Qdrant. These chunks form the context.
        - The compiled context and `user_message` are then sent to the configured LLM (OpenAI/Claude) for generating `chatbot_response`.
        - The LLM is instructed to be concise and answer only based on the provided context.
    - **Error Handling**: Returns `500 Internal Server Error` on LLM API or context retrieval failures.

### Qdrant Vector DB Integration
- **Deployment**: Utilize Qdrant Cloud Free Tier.
- **Collections**: A single collection named `textbook_chunks` will store all embeddings.
- **Schema**: Each point in Qdrant will include a vector (embedding) and a payload with `text` (original chunk), `chapter_title`, `file_path`, `chunk_id`, and `page_number`.

### Neon Postgres for Session Logs
- **Deployment**: Utilize Neon Postgres (Serverless Postgres).
- **Purpose**: Store chat session history, user queries, chatbot responses, and any errors for debugging, analytics, and audit purposes.
- **Schema (Example)**:
    ```sql
    CREATE TABLE chat_logs (
        id UUID PRIMARY KEY,
        timestamp TIMESTAMPTZ DEFAULT NOW(),
        user_message TEXT NOT NULL,
        context_text TEXT,
        chatbot_response TEXT NOT NULL,
        sources TEXT[],
        embedding_model VARCHAR(50),
        llm_model VARCHAR(50),
        error_message TEXT
    );
    ```

### Embedding Logic
- **Chunking (`backend/app/utils/markdown_parser.py`)**:
    - Markdown files are parsed and split into smaller, semantically coherent text chunks.
    - Chunk size and overlap will be configurable (e.g., 500 tokens with 100 token overlap).
    - Code blocks and MCQs within markdown should ideally be treated as separate, distinct chunks or handled specially.
- **Cleaning**: Basic text cleaning (e.g., removing excessive whitespace) before embedding.
- **Markdown Loader**: A utility to read `.md` files and extract content.
- **OpenAI/Claude Embeddings**:
    - The `backend/app/services/embedding_service.py` will abstract interaction with OpenAI's embedding API (e.g., `text-embedding-ada-002` or newer) or Claude's embedding service.
    - Selection between OpenAI/Claude will be via environment variable.

### RAG Flow
1.  **Retrieve**: Based on `user_message` (or `context_text` if provided), search Qdrant for top-N relevant text chunks.
2.  **Context Assembly**: Concatenate retrieved chunks (or `context_text`) to form a coherent context window for the LLM.
3.  **LLM Call**: Send `user_message` and assembled `context` to the LLM (OpenAI/Claude API).
4.  **Response Generation**: LLM generates `chatbot_response` based *strictly* on the provided context.
5.  **Logging**: Log the full interaction (user message, context, response, models used) to Neon Postgres.

### Error Handling and Logging
- **Centralized Error Handling**: FastAPI exception handlers for common errors (e.g., `HTTPException`).
- **Structured Logging**: Use `logging` module with JSON format to Neon.
- **Health Checks**: `/health` endpoint to check backend service status, Qdrant connectivity, and LLM API reachability.

---

## 4. Frontend Specification

The frontend will be built with Docusaurus v3, serving as the textbook and hosting the RAG chat widget.

### Docusaurus v3 Structure
- Standard Docusaurus project setup.
- `frontend/docs/` will contain all markdown chapters, rendered by Docusaurus.
- Custom theming (if necessary) in `frontend/src/css/`.

### React MDX Component for RAG Chat Widget
- A React component (`frontend/src/components/RAGChatWidget.js/tsx`) will implement the chat UI.
- This component will be able to receive selected text as a prop or through a browser event listener.
- It will include:
    - Input field for user questions.
    - Display area for chat history (user questions and AI responses).
    - Send button.
    - Loading indicator.
    - "Selected text" display area when context is active.

### Selected-Text Only Answering Mode
- **Mechanism**: A JavaScript listener will detect text selection events (`mouseup`, `touchend`) within the Docusaurus content area.
- When text is selected, the `RAGChatWidget` will be programmatically opened or a UI hint will appear, pre-filling the `context_text` field in the chat input with the selected text.
- The user's question will then be explicitly directed to the `/chat` endpoint with this `context_text`.

### Instructions to Embed Widget in All Doc Pages
- The `RAGChatWidget` will be integrated globally into the Docusaurus template, likely within `src/theme/DocItem/Content/index.js` or a similar wrapper, to ensure it appears on all textbook pages without manual embedding in each markdown file.
- The widget will initially be hidden and appear on text selection or a specific user action (e.g., a floating button).

---

## 5. Automation & Deployment

All build, deployment, and indexing processes will be fully automated using GitHub Actions.

### Scripts for Auto-Indexing Markdown Chapters into Qdrant
- A Python script (`backend/scripts/index_markdown.py`) will be executed by the GitHub Action.
- This script will:
    - Iterate through all `.md` files in `frontend/docs/`.
    - Call the `POST /index` endpoint for each new or modified markdown file.
    - Maintain a record of indexed files to avoid redundant indexing.

### GitHub Actions
- **`frontend.yml` (Docusaurus to GitHub Pages)**:
    - **Trigger**: `push` to `main` branch (path filter `frontend/**`).
    - **Jobs**:
        - `build_and_deploy`:
            - Checkout repository.
            - Setup Node.js.
            - Install frontend dependencies (`yarn install`).
            - Build Docusaurus site (`yarn build`).
            - Deploy built site to `gh-pages` branch using `peaceiris/actions-gh-pages`.

- **`backend.yml` (FastAPI Backend Deployment & Indexing)**:
    - **Trigger**: `push` to `main` branch (path filter `backend/**`, `frontend/docs/**`).
    - **Jobs**:
        - `build_and_deploy_backend`:
            - Checkout repository.
            - Docker login to container registry.
            - Build Docker image for `backend/` (`docker build -t your_repo/physical-ai-rag:latest .`).
            - Push Docker image.
            - Deploy to chosen cloud provider (e.g., Render/Fly.io) using their respective deployment actions or a custom script.
            - **Environment Variables**: Passed securely from GitHub Secrets.
        - `index_markdown_content` (depends on `build_and_deploy_backend`):
            - Only runs if changes detected in `frontend/docs/`.
            - Executes the `backend/scripts/index_markdown.py` script via a `curl` command to the deployed `/index` endpoint or by running the script within the CI/CD environment with access to Qdrant.
            - Requires access to `OPENAI_API_KEY` (or `CLAUDE_API_KEY`), `QDRANT_HOST`, `QDRANT_PORT`.

### Environment Variables Structure (`.env.example`)
Located at `backend/.env.example`, this file will serve as a template for required environment variables.

```ini
# OpenAI API Key (if using OpenAI)
OPENAI_API_KEY="your_openai_api_key_here"

# Claude API Key (if using Claude, uncomment and set)
# CLAUDE_API_KEY="your_claude_api_key_here"

# Qdrant Cloud connection details
QDRANT_HOST="<your-qdrant-cluster-url>"
QDRANT_PORT="443" # Or relevant port
QDRANT_API_KEY="your_qdrant_api_key_here" # For cloud deployments

# Neon Postgres connection details
DATABASE_URL="postgresql://user:password@host:port/database" # Full connection string

# FastAPI settings
FASTAPI_HOST="0.0.0.0"
FASTAPI_PORT="8000"

# Optional: For local development/testing
# EMBEDDING_MODEL_NAME="text-embedding-ada-002"
# LLM_MODEL_NAME="gpt-4-turbo"
```

---

## 6. Quality & Standards

Adherence to the project constitution's principles is paramount to ensure a high-quality, maintainable, and robust system.

-   **Modular, Clean, Production-Ready Code**:
    -   Code will be organized into logical modules (`backend/app/api`, `backend/app/services`, `frontend/src/components`).
    -   Follow Python (PEP 8) and JavaScript/TypeScript (Prettier, ESLint) style guides.
    -   Comprehensive unit and integration tests for backend components, especially RAG logic and API endpoints.
    -   Docstrings and comments will explain complex logic, but code should be self-documenting where possible.
-   **Repeatable Builds**:
    -   All dependencies will be explicitly managed (`requirements.txt`, `package.json`, `yarn.lock`).
    -   Docker will be used for backend containerization to ensure consistent environments.
    -   GitHub Actions will guarantee consistent build and deployment processes.
-   **Full Documentation for Each Component**:
    -   API documentation (FastAPI automatically generates OpenAPI docs).
    -   READMEs for `backend/` and `frontend/` explaining how to run/develop locally.
    -   `history/prompts/` and `history/adr/` will be diligently maintained as per Spec-Kit Plus guidelines.
-   **Consistent Naming Conventions and Folder Organization**:
    -   As defined in Section 1. Folder Structure and throughout this document.

---

## 7. Data Flow / RAG Logic Diagram

```mermaid
graph TD
    A[User Query] --> B{Selected Text Provided?};
    B -- Yes --> C[Use Selected Text as Context];
    B -- No --> D[Embed User Query];
    D --> E[Qdrant Vector Search];
    E --> F[Retrieve Top-N Chunks];
    C --> G[Assemble Context];
    F --> G;
    G --> H{LLM (OpenAI/Claude) Call};
    H --> I[LLM Response];
    I --> J[Chatbot Response to User];
    K[Neon Postgres Logging] --> J;
    H --> K;
    E --> K;
```

---

This technical specification provides a complete and actionable blueprint for AI-driven implementation of the "Physical AI & Humanoid Robotics – Textbook + RAG System" project.
