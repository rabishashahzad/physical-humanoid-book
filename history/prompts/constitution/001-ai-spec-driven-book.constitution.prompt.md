---
id: 001
title: AI Spec Driven Book Constitution
stage: constitution
date: 2025-12-17
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: lenovo
command: /sp.constitution
labels: ["constitution", "spec-driven", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

/sp.constitution /sp.constitution

Project: AI/Spec-Driven Book + RAG Chatbot

Purpose:
AI-authored technical book via Spec-Kit Plus & Claude Code, published on Docusaurus,
with embedded RAG chatbot answering only from book content.

Core Principles:
- Spec-driven, accurate, traceable, modular, deterministic
- Security & privacy enforced
- Developer-focused tone

Content Standards:
- Clear chapters with objectives, scope, inputs, outputs, acceptance criteria
- Runnable, documented code examples
- No placeholders or hallucinations

AI Authoring:
- Claude Code generates structured content per spec
- Spec-Kit Plus validates each chapter
- Acceptance criteria required for all outputs

RAG Chatbot:
- FastAPI backend, OpenAI Agents/ChatKit, Neon Postgres, Qdrant Free Tier
- Sources: book content only
- Responses cite book sections; refuse out-of-scope queries
- Selected-text retrieval supported

Constraints:
- Deploy on GitHub Pages
- Free-tier infrastructure only
- Reproducible setup, deterministic embeddings

Success Criteria:
- Book published and functional
- Chatbot answers grounded in book
- Zero hallucinations
- Spec-Kit Plus validation passed

## Response snapshot

Updated the project constitution file (.specify/memory/constitution.md) with project-specific values for the AI/Spec-Driven Book + RAG Chatbot project. The constitution now includes six core principles: Spec-Driven Development First, Authoritative Source Mandate, Test-First Approach, Knowledge Capture and Documentation, Security & Privacy Enforcement, and Deterministic and Reproducible Systems. Additional sections include constraints, development workflow, and governance rules.

## Outcome

- ✅ Impact: Created initial project constitution with all core principles defined
- 🧪 Tests: No tests required for this configuration change
- 📁 Files: Updated .specify/memory/constitution.md with project-specific values
- 🔁 Next prompts: Ready to proceed with spec, plan, and implementation phases
- 🧠 Reflection: Constitution provides clear governance framework for project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin feature specification using /sp.specify