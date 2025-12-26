# Implementation Plan: Book RAG Chatbot with Translation

**Branch**: `002-book-rag-chatbot` | **Date**: 2025-12-20 | **Spec**: [specs/002-book-rag-chatbot/spec.md](spec.md)

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a Retrieval-Augmented Generation (RAG) chatbot embedded in a React-based book frontend with English to Urdu translation capabilities. The system uses Qdrant Cloud for vector storage, FastAPI backend, Neon Postgres for authentication and chat history, with strict adherence to book content for responses (no hallucination).

## Technical Context

<!--
  ACTION REQUIRED: Replace the content in this section with the technical details
  for the project. The structure here is presented in advisory capacity to guide
  the iteration process.
-->

**Language/Version**: Python 3.11 (FastAPI backend), TypeScript/JavaScript (React frontend), Node.js 18+
**Primary Dependencies**: FastAPI, Qdrant, Neon Postgres, React, OpenAI SDK, BetterAuth, Transformers (for translation)
**Storage**: Neon Serverless Postgres (user data, chat history), Qdrant Cloud (vector embeddings)
**Testing**: pytest (backend), Jest/React Testing Library (frontend)
**Target Platform**: Web application (React frontend), Linux server (FastAPI backend)
**Project Type**: web - determines source structure
**Performance Goals**: Response time under 5 seconds, handle 1000 concurrent users
**Constraints**: Free-tier infrastructure only, <200ms p95 for internal operations, no external knowledge in responses
**Scale/Scope**: Multi-book support, 10k+ users, isolated knowledge bases per book/module

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

1. **RAG Knowledge Source Constraint**: System must strictly limit responses to book content only, responding with 'یہ معلومات کتاب میں موجود نہیں ہے۔' when information is not found in the book.
2. **Language Support Requirements**: System must support English as default and Urdu as translation language only, with Hindi vocabulary strictly forbidden.
3. **Security & Privacy Enforcement**: No hardcoded secrets or tokens; use secure storage mechanisms for authentication and API keys.
4. **Test-First Approach**: All implementation work must include tests written before code.
5. **Deterministic and Reproducible Systems**: Infrastructure and deployments must follow idempotent patterns.

## Project Structure

### Documentation (this feature)

```text
specs/002-book-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── user.py
│   │   ├── chat_session.py
│   │   ├── message.py
│   │   ├── book_content.py
│   │   └── knowledge_base.py
│   ├── services/
│   │   ├── auth_service.py
│   │   ├── rag_service.py
│   │   ├── translation_service.py
│   │   └── embedding_service.py
│   ├── api/
│   │   ├── auth_routes.py
│   │   ├── chat_routes.py
│   │   ├── translation_routes.py
│   │   └── book_routes.py
│   └── main.py
└── tests/
    ├── unit/
    ├── integration/
    └── contract/

frontend/
├── src/
│   ├── components/
│   │   ├── ChatInterface/
│   │   ├── BookViewer/
│   │   ├── TranslationToggle/
│   │   └── Auth/
│   ├── services/
│   │   ├── api.js
│   │   ├── auth.js
│   │   └── translation.js
│   ├── pages/
│   │   ├── BookPage.js
│   │   └── ChatPage.js
│   └── utils/
│       ├── textSelection.js
│       └── constants.js
├── public/
└── tests/
    ├── unit/
    └── integration/
```

**Structure Decision**: Selected web application structure with separate backend and frontend to handle the RAG functionality, authentication, and React-based book frontend as specified in the requirements. The backend handles RAG processing, authentication, and translation, while the frontend provides the user interface with red chat board UI.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multiple external services | Required by specification (Qdrant Cloud, Neon Postgres, OpenAI) | Using single database would not meet RAG requirements |