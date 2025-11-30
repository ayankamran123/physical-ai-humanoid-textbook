# Feature: Voice-Activated RAG

## 1. Goal
Add a Microphone button to the Chat component for Speech-to-Text.

## 2. Requirements
- Use Web Speech API.
- Add a Mic icon next to the input.
- IMPORTANT: Ensure 'handleSendMessage' is defined BEFORE 'handleVoiceInput' to avoid initialization errors.
- Add Text-to-Speech (Speaker icon) for AI responses.