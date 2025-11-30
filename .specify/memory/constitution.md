<!--
Sync Impact Report:
Version change: 0.0.0 -> 1.0.0 (MAJOR: Initial creation based on user provided constitution)
Modified principles:
  - PRINCIPLE_1_NAME -> I. AI-first
  - PRINCIPLE_2_NAME -> II. Modular, Repeatable, Documented
  - PRINCIPLE_3_NAME -> III. Zero Manual Coding
  - PRINCIPLE_4_NAME -> IV. Clear Naming & Folder Structure
Added sections:
  - Project Mission & Objectives
  - Project Scope & Success Criteria
Removed sections:
  - [PRINCIPLE_5_NAME] (empty template section removed)
  - [PRINCIPLE_6_NAME] (empty template section removed)
  - [PRINCIPLE__DESCRIPTION] (empty template section removed)
Templates requiring updates:
  - .specify/templates/plan-template.md (⚠ pending)
  - .specify/templates/spec-template.md (⚠ pending)
  - .specify/templates/tasks-template.md (⚠ pending)
  - .specify/templates/commands/sp.constitution.md (⚠ pending)
  - .specify/templates/commands/sp.phr.md (⚠ pending)
  - README.md (⚠ pending)
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics – Textbook + RAG System Constitution

## Core Principles

### I. AI-first
Human input is limited to providing specifications; AI is responsible for generating the core content and implementation.

### II. Modular, Repeatable, Documented
All components and processes must be modular, easily repeatable, and thoroughly documented to ensure maintainability and scalability.

### III. Zero Manual Coding
The development process must strive for zero manual coding, leveraging AI and automation for all code generation and integration.

### IV. Clear Naming & Folder Structure
Maintain a consistent, logical, and intuitive naming convention and folder structure across the entire project for enhanced clarity and navigability.

## Project Mission & Objectives

### Mission
Create an AI-generated university-level textbook on Physical AI & Humanoid Robotics using Spec-Kit Plus & Claude Code, deployed with Docusaurus, integrated with a RAG chatbot (FastAPI, Qdrant, Neon, OpenAI/Claude), answering questions including selected-text only.

### Objectives
- Generate full textbook with AI
- Deploy via Docusaurus → GitHub Pages
- Implement RAG backend (embedding, indexing, retrieval, chat)
- Embed chat widget in book
- Automate indexing, build, and deployment
- Maintain clean, modular, production-ready code

## Project Scope & Success Criteria

### Scope
- **Book:** 8–10 chapters: theory, diagrams, Python/ROS2 code, MCQs
- **Frontend:** Docusaurus v3 + chat widget, Selected-text only answering mode
- **Backend:** FastAPI: /index, /search, /chat, Qdrant vector DB + Neon logs, Embeddings via OpenAI/Claude, Markdown loader + chunking
- **Automation:** Auto-index markdown, GitHub Actions for frontend & backend, .env.example for variables

### Success Criteria
- Book deployed & readable
- RAG chatbot works
- Selected-text answering works
- Qdrant indexing successful
- Backend deployed & logs stored

## Governance
Amendments to this constitution require a documented proposal, team review, and approval. All pull requests and code reviews must verify compliance with these principles. New features and architectural decisions must explicitly align with the principles outlined herein.

**Version**: 1.0.0 | **Ratified**: 2025-11-28 | **Last Amended**: 2025-11-28
